import time
import sys
import io
import contextlib
from collections import deque

import numpy as np
import dxcam
import serial
import colorsys

# ========= COLOR TUNING =========
BRIGHTNESS = 0.9      # 0.0–1.0, overall strength
SATURATION_BOOST = 1.5  # >1.0 = more saturated
MIN_VISIBLE = 10      # anything below this ends up as 0
# ===============================



# ========= CONFIG =========
TOP_LEDS    = 51
RIGHT_LEDS  = 22
BOTTOM_LEDS = 51
LEFT_LEDS   = 23

NUM_LEDS = TOP_LEDS + RIGHT_LEDS + BOTTOM_LEDS + LEFT_LEDS

TOP_BOTTOM_STRIP_HEIGHT = 40   # px

COM_PORT = "COM3"        # <-- adjust if needed
BAUD     = 2_000_000     # <-- high baud so we can send full frames
START_BYTE = 255         # 0xFF start marker
# =========================


@contextlib.contextmanager
def suppress_stderr():
    """Hide dxcam/comtypes spam in the console."""
    old_stderr = sys.stderr
    sys.stderr = io.StringIO()
    try:
        yield
    finally:
        sys.stderr = old_stderr

def apply_color_correction(led_float: np.ndarray) -> np.ndarray:
    """
    led_float: (NUM_LEDS, 3) float32 in 0..255
    Returns: (NUM_LEDS, 3) uint8 after saturation + brightness + min visible.
    """
    # clamp to [0,255] and normalize to 0..1
    arr = np.clip(led_float, 0, 255).astype(np.float32) / 255.0

    out = np.empty_like(arr)

    for i, (r, g, b) in enumerate(arr):
        h, s, v = colorsys.rgb_to_hsv(r, g, b)

        # boost saturation
        s = min(1.0, s * SATURATION_BOOST)

        # apply brightness
        v = v * BRIGHTNESS

        # min visible threshold
        if v < MIN_VISIBLE / 255.0:
            v = 0.0

        r2, g2, b2 = colorsys.hsv_to_rgb(h, s, v)
        out[i] = (r2, g2, b2)

    # back to 0..255 uint8
    return (out * 255.0).astype(np.uint8)


def regions_top_bottom_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    """
    Vectorized: split horizontal strip into num_regions vertical chunks
    and return (num_regions, 3) average RGB (float32).
    """
    h, w, _ = strip_img.shape
    region_w = w // num_regions      # integer; may ignore a few rightmost columns
    w_used = region_w * num_regions
    if w_used == 0:
        raise ValueError("region_w became 0; check screen width vs num_regions")

    strip = strip_img[:, :w_used, :].astype(np.float32)   # (h, w_used, 3)
    strip = strip.reshape(h, num_regions, region_w, 3)
    return strip.mean(axis=(0, 2))  # (num_regions, 3)


def regions_left_right_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    """
    Vectorized: split vertical strip into num_regions horizontal chunks
    and return (num_regions, 3) average RGB (float32).
    """
    h, w, _ = strip_img.shape
    region_h = h // num_regions
    h_used = region_h * num_regions
    if h_used == 0:
        raise ValueError("region_h became 0; check strip height vs num_regions")

    strip = strip_img[:h_used, :, :].astype(np.float32)   # (h_used, w, 3)
    strip = strip.reshape(num_regions, region_h, w, 3)
    return strip.mean(axis=(1, 2))  # (num_regions, 3)


# ---------- NEW: serial helpers ----------

def open_serial():
    try:
        ser = serial.Serial(COM_PORT, BAUD, timeout=0)
        print(f"Opened serial {COM_PORT} @ {BAUD}")
        return ser
    except Exception as e:
        print(f"Could not open serial port {COM_PORT}: {e}")
        return None


def send_frame(ser: serial.Serial | None, led_colors: np.ndarray):
    """
    Send one frame to Arduino.
    led_colors: (NUM_LEDS, 3) uint8 RGB
    Frame format: 0xFF + NUM_LEDS*3 bytes
    """
    if ser is None:
        return

    # Make sure shape & dtype are correct
    if led_colors.shape != (NUM_LEDS, 3) or led_colors.dtype != np.uint8:
        raise ValueError("led_colors must be (NUM_LEDS,3) uint8")

    try:
        # Build frame: start byte + raw RGB bytes
        frame = bytes([START_BYTE]) + led_colors.tobytes()
        ser.write(frame)
        # no flush: let OS buffer handle it
    except Exception as e:
        # If something goes wrong, print once and disable further sending
        print(f"Serial write error: {e}")
        return


# ----------------------------------------


def main():
    with suppress_stderr():
        camera = dxcam.create(output_color="RGB")

    # Start continuous capture at 60 target fps
    camera.start(target_fps=60, video_mode=True)

    # Open serial
    ser = open_serial()

    screen_w = None
    screen_h = None
    slices_ready = False

    frame_times = deque()
    last_print_time = time.perf_counter()

    # preallocate LED buffers
    led_float = np.zeros((NUM_LEDS, 3), dtype=np.float32)
    led_colors = np.zeros((NUM_LEDS, 3), dtype=np.uint8)

    print("Capturing... press Ctrl+C to stop.\n")

    try:
        while True:
            frame = camera.get_latest_frame()
            if frame is None:
                continue

            if not slices_ready:
                # First good frame → compute geometry once
                h, w, _ = frame.shape
                screen_w, screen_h = w, h
                strip_h = TOP_BOTTOM_STRIP_HEIGHT

                # Top & bottom strip y-ranges
                top_y0, top_y1 = 0, strip_h
                bottom_y0, bottom_y1 = screen_h - strip_h, screen_h

                # Side strip width = width of one top region (approx)
                top_region_width = screen_w / TOP_LEDS
                side_w = int(top_region_width)

                # Left & right x-ranges
                left_x0, left_x1 = 0, side_w
                right_x0, right_x1 = screen_w - side_w, screen_w

                # Vertical range for side strips (skip top/bottom corners)
                side_y0, side_y1 = strip_h, screen_h - strip_h

                geom = {
                    "screen_w": screen_w,
                    "screen_h": screen_h,
                    "strip_h": strip_h,
                    "top": (top_y0, top_y1),
                    "bottom": (bottom_y0, bottom_y1),
                    "left_x": (left_x0, left_x1),
                    "right_x": (right_x0, right_x1),
                    "side_y": (side_y0, side_y1),
                    "side_w": side_w,
                }

                print("Geometry:", geom, "\n")
                slices_ready = True

            strip_h = TOP_BOTTOM_STRIP_HEIGHT

            # --- Slice strips from the full frame ---
            # top & bottom: full width
            top_img = frame[0:strip_h, :, :]
            bottom_img = frame[screen_h - strip_h:screen_h, :, :]

            # side strips: narrow width, between top & bottom strips
            top_region_width = screen_w / TOP_LEDS
            side_w = int(top_region_width)

            left_img = frame[strip_h:screen_h - strip_h, 0:side_w, :]
            right_img = frame[strip_h:screen_h - strip_h,
                              screen_w - side_w:screen_w, :]

            # --- Compute per-region averages (float32 RGB), vectorized ---
            top_means = regions_top_bottom_vec(top_img, TOP_LEDS)
            bottom_means = regions_top_bottom_vec(bottom_img, BOTTOM_LEDS)
            right_means = regions_left_right_vec(right_img, RIGHT_LEDS)
            left_means = regions_left_right_vec(left_img, LEFT_LEDS)

            # --- STACK INTO LED ORDER (clockwise) ---
            idx = 0
            led_float[idx:idx + TOP_LEDS] = top_means
            idx += TOP_LEDS

            led_float[idx:idx + RIGHT_LEDS] = right_means
            idx += RIGHT_LEDS

            led_float[idx:idx + BOTTOM_LEDS] = bottom_means[::-1]
            idx += BOTTOM_LEDS

            led_float[idx:idx + LEFT_LEDS] = left_means[::-1]

            # apply color correction & convert to uint8
            led_colors[:] = apply_color_correction(led_float)

            # --- SEND TO ARDUINO ---
            send_frame(ser, led_colors)

            # --- FPS over last 1 second ---
            now = time.perf_counter()
            frame_times.append(now)
            while frame_times and (now - frame_times[0]) > 1.0:
                frame_times.popleft()
            fps = len(frame_times)

            if now - last_print_time >= 1.0:
                last_print_time = now
                # print(
                #     f"FPS ~ {fps:3d} | "
                #     f"LED[0]={led_colors[0]} "
                #     f"LED[{TOP_LEDS - 1}]={led_colors[TOP_LEDS - 1]} "
                #     f"LED[{TOP_LEDS}]={led_colors[TOP_LEDS]} "
                #     f"LED[{TOP_LEDS + RIGHT_LEDS - 1}]={led_colors[TOP_LEDS + RIGHT_LEDS - 1]}"
                # )

    except KeyboardInterrupt:
        print("\nStopped by user.")
        camera.stop()
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()
