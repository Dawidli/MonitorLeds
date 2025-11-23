import time
import sys
import io
import contextlib
import threading
import queue
from typing import Optional

import numpy as np
import serial
import colorsys

# Try high-performance C++ capture first, fall back to Python dxcam if not installed
try:
    import dxcam_cpp as dxcam
except ImportError:
    import dxcam


# ========= LED / LAYOUT CONFIG =========
# LED counts per edge (must match your physical strip)
TOP_LEDS    = 51
RIGHT_LEDS  = 22
BOTTOM_LEDS = 51
LEFT_LEDS   = 23

NUM_LEDS = TOP_LEDS + RIGHT_LEDS + BOTTOM_LEDS + LEFT_LEDS

# Height in pixels for top/bottom capture strips
TOP_BOTTOM_STRIP_HEIGHT = 40
# ======================================


# ========= SERIAL CONFIG =========
COM_PORT   = "COM3"      # change if your Arduino appears on another port
BAUD       = 2_000_000   # must match Serial.begin() on Arduino
START_BYTE = 255         # simple start-of-frame marker
# ================================


# ========= COLOR / SMOOTHING TUNING =========
BRIGHTNESS       = 0.9    # overall brightness scale (0.0–1.0)
SATURATION_BOOST = 1.6    # >1.0 = more saturated colors
MIN_VISIBLE      = 10     # below this (0–255) → treat as black/off

SMOOTHING_ALPHA  = 0.6    # temporal smoothing (0..1): higher = smoother/slower
GAMMA            = 2.0    # gamma correction on brightness

# Warm / cool tint:
#   0.0 = neutral
#   +0.5 = noticeably warmer (more red, less blue)
#   -0.5 = noticeably cooler (more blue, less red)
WARM_COOL        = 0.0
# ===========================================


# ========= PILLARBOX / YOUTUBE MODE =========
# When True, we assume 16:9 content centered on a wider screen (e.g. 21:9),
# and ignore the side black bars by sampling only inside the active 16:9 area.
PILLARBOX_MODE = False
# ===========================================


@contextlib.contextmanager
def suppress_stderr():
    """Temporarily hide dxcam/comtypes noise on stderr."""
    old_stderr = sys.stderr
    sys.stderr = io.StringIO()
    try:
        yield
    finally:
        sys.stderr = old_stderr


def regions_top_bottom_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    """
    Compute average color per region for a horizontal strip (top or bottom).

    strip_img: (H, W, 3)
    Returns: (num_regions, 3), float32, RGB in 0..255
    """
    h, w, _ = strip_img.shape
    region_w = w // num_regions          # integer width per region
    w_used   = region_w * num_regions    # drop any leftover rightmost columns

    if w_used == 0:
        raise ValueError("region_w became 0; check screen width vs num_regions")

    strip = strip_img[:, :w_used, :].astype(np.float32)      # (h, w_used, 3)
    strip = strip.reshape(h, num_regions, region_w, 3)       # (h, N, w_per, 3)
    return strip.mean(axis=(0, 2))                           # (N, 3)


def regions_left_right_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    """
    Compute average color per region for a vertical strip (left or right).

    strip_img: (H, W, 3)
    Returns: (num_regions, 3), float32, RGB in 0..255
    """
    h, w, _ = strip_img.shape
    region_h = h // num_regions
    h_used   = region_h * num_regions

    if h_used == 0:
        raise ValueError("region_h became 0; check strip height vs num_regions")

    strip = strip_img[:h_used, :, :].astype(np.float32)      # (h_used, w, 3)
    strip = strip.reshape(num_regions, region_h, w, 3)       # (N, h_per, w, 3)
    return strip.mean(axis=(1, 2))                           # (N, 3)


def open_serial() -> Optional[serial.Serial]:
    """
    Try to open the serial port for the Arduino.
    Returns a Serial object or None if opening fails.
    """
    try:
        ser = serial.Serial(COM_PORT, BAUD, timeout=0)
        return ser
    except Exception as e:
        print(f"Warning: could not open serial port {COM_PORT}: {e}")
        return None


def serial_worker(
    ser: Optional[serial.Serial],
    q: "queue.Queue[np.ndarray]",
    stop_evt: threading.Event,
):
    """
    Background thread that sends LED frames to the Arduino.

    It expects (NUM_LEDS, 3) uint8 RGB arrays in `q`.
    Using a separate thread decouples USB timing from the capture loop.
    """
    if ser is None:
        # If no serial is available, just wait for stop signal and ignore frames.
        while not stop_evt.is_set():
            try:
                item = q.get(timeout=0.1)
                if item is None:
                    break
            except queue.Empty:
                pass
        return

    try:
        while not stop_evt.is_set():
            try:
                led_colors = q.get(timeout=0.1)
            except queue.Empty:
                continue

            if led_colors is None:
                # Sentinel to shut down the worker
                break

            frame = bytes([START_BYTE]) + led_colors.tobytes()
            try:
                ser.write(frame)
            except Exception as e:
                print(f"Serial write error: {e}")
                # We keep going; worst case LEDs freeze on last frame
    finally:
        if ser is not None and ser.is_open:
            ser.close()


def apply_color_correction(led_float: np.ndarray) -> np.ndarray:
    """
    Apply saturation, brightness, gamma, warm/cool tint, and min-visible threshold.

    Input:
        led_float: (NUM_LEDS, 3) float32 array, values in [0, 255]
    Output:
        (NUM_LEDS, 3) uint8 array, values in [0, 255]
    """
    # Normalize to 0..1
    arr = np.clip(led_float, 0, 255).astype(np.float32) / 255.0
    out = np.empty_like(arr)

    for i, (r, g, b) in enumerate(arr):
        # Work in HSV for saturation & brightness/gamma
        h, s, v = colorsys.rgb_to_hsv(r, g, b)

        # Boost saturation
        s = min(1.0, s * SATURATION_BOOST)

        # Apply brightness scaling
        v *= BRIGHTNESS

        # Gamma correction on brightness
        if v > 0.0:
            v = v ** GAMMA

        # Min visible threshold: below this, turn the LED off
        if v < MIN_VISIBLE / 255.0:
            v = 0.0

        r2, g2, b2 = colorsys.hsv_to_rgb(h, s, v)

        # Warm/cool tint in RGB space:
        # Positive WARM_COOL -> more red / less blue (warmer)
        # Negative WARM_COOL -> more blue / less red (cooler)
        if WARM_COOL != 0.0:
            k = 0.15 * WARM_COOL  # strength of tint
            r2 = float(np.clip(r2 + k, 0.0, 1.0))
            b2 = float(np.clip(b2 - k, 0.0, 1.0))
            # We leave G unchanged to keep things natural

        out[i] = (r2, g2, b2)

    # Back to 0..255 uint8
    return (out * 255.0).astype(np.uint8)


def main():
    # Create camera with stderr suppressed to avoid comtypes spam in console
    with suppress_stderr():
        camera = dxcam.create(output_color="RGB")

    # Start the capture thread inside dxcam:
    camera.start(target_fps=120, video_mode=True)

    # Set up serial + background worker
    ser = open_serial()
    serial_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
    stop_evt = threading.Event()

    worker = threading.Thread(
        target=serial_worker,
        args=(ser, serial_queue, stop_evt),
        daemon=True,
    )
    worker.start()

    screen_w = None
    screen_h = None
    slices_ready = False

    # Pillarbox offset in pixels (computed after first frame)
    pillar_pixels = 0

    # Buffers reused every frame to avoid re-allocations
    led_float          = np.zeros((NUM_LEDS, 3), dtype=np.float32)
    smoothed_led_float = np.zeros((NUM_LEDS, 3), dtype=np.float32)
    have_smoothed      = False
    led_colors         = np.zeros((NUM_LEDS, 3), dtype=np.uint8)

    try:
        while True:
            frame = camera.get_latest_frame()
            if frame is None:
                continue

            # On first valid frame, detect screen geometry and compute pillarbox offset
            if not slices_ready:
                h, w, _ = frame.shape
                screen_w, screen_h = w, h

                if PILLARBOX_MODE:
                    # Assume 16:9 content centered, full screen height
                    active_width_16_9 = int(screen_h * 16 / 9)
                    if active_width_16_9 < screen_w:
                        # Side bar width in pixels
                        pillar_pixels = (screen_w - active_width_16_9) // 2
                    else:
                        pillar_pixels = 0
                else:
                    pillar_pixels = 0

                # Safety: bars should never exceed half width
                if pillar_pixels * 2 > screen_w:
                    pillar_pixels = 0

                slices_ready = True

            strip_h = TOP_BOTTOM_STRIP_HEIGHT

            # --- Compute horizontal active region (for pillarbox mode) ---
            if pillar_pixels > 0:
                # Active 16:9 area horizontally
                active_left  = pillar_pixels
                active_right = screen_w - pillar_pixels
                active_width = active_right - active_left
            else:
                # No pillarbox: active area is entire width
                active_left  = 0
                active_right = screen_w
                active_width = screen_w

            # --- Slice top & bottom strips using active horizontal area ---
            top_img    = frame[0:strip_h, active_left:active_right, :]
            bottom_img = frame[screen_h - strip_h:screen_h, active_left:active_right, :]

            # --- Side strips ---
            # Side strip width is based on active width, not full screen width.
            top_region_width = active_width / TOP_LEDS
            side_w = int(top_region_width)
            if side_w <= 0:
                side_w = 1

            # Ensure we have room for side_w on each side
            if active_width < 2 * side_w:
                # Fallback: use very narrow side strips
                side_w = max(1, active_width // 4)

            # Left and right strip x ranges within the active area
            left_x_start  = active_left
            left_x_end    = left_x_start + side_w
            right_x_end   = active_right
            right_x_start = right_x_end - side_w

            # Clamp ranges to screen bounds (safety)
            left_x_start  = max(0, left_x_start)
            left_x_end    = min(screen_w, left_x_end)
            right_x_start = max(0, right_x_start)
            right_x_end   = min(screen_w, right_x_end)

            # Vertical range for side strips (skip top/bottom strip height)
            v_top    = strip_h
            v_bottom = screen_h - strip_h

            left_img = frame[v_top:v_bottom, left_x_start:left_x_end, :]
            right_img = frame[v_top:v_bottom, right_x_start:right_x_end, :]

            # --- Compute average color per region (float32 RGB) ---
            top_means    = regions_top_bottom_vec(top_img, TOP_LEDS)
            bottom_means = regions_top_bottom_vec(bottom_img, BOTTOM_LEDS)
            right_means  = regions_left_right_vec(right_img, RIGHT_LEDS)
            left_means   = regions_left_right_vec(left_img, LEFT_LEDS)

            # --- Stack LEDs in physical order (clockwise around screen) ---
            # 1) Top:    left -> right
            # 2) Right:  top -> bottom
            # 3) Bottom: right -> left (reversed)
            # 4) Left:   bottom -> top (reversed)
            idx = 0
            led_float[idx:idx + TOP_LEDS] = top_means
            idx += TOP_LEDS

            led_float[idx:idx + RIGHT_LEDS] = right_means
            idx += RIGHT_LEDS

            led_float[idx:idx + BOTTOM_LEDS] = bottom_means[::-1]
            idx += BOTTOM_LEDS

            led_float[idx:idx + LEFT_LEDS] = left_means[::-1]

            # --- Temporal smoothing (exponential moving average) ---
            if not have_smoothed:
                smoothed_led_float[:] = led_float
                have_smoothed = True
            else:
                smoothed_led_float[:] = (
                    SMOOTHING_ALPHA * smoothed_led_float
                    + (1.0 - SMOOTHING_ALPHA) * led_float
                )

            # --- Apply color correction, gamma, warm/cool etc. ---
            led_colors[:] = apply_color_correction(smoothed_led_float)

            # --- Queue frame for serial worker ---
            # If LEDs are completely black, skip sending to avoid tiny black blips.
            if np.any(led_colors):
                try:
                    # Queue size is 1; if worker is busy, drop this frame
                    serial_queue.put_nowait(led_colors.copy())
                except queue.Full:
                    pass


    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        # Stop capture
        camera.stop()
        # Optionally send a single all-black frame (Arduino will fade from last colors anyway)
        try:
            off_frame = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
            serial_queue.put(off_frame, timeout=0.1)
        except queue.Full:
            pass
        # Tell worker to exit
        stop_evt.set()
        try:
            serial_queue.put_nowait(None)  # sentinel value
        except queue.Full:
            pass
        worker.join(timeout=1.0)


if __name__ == "__main__":
    main()
