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

try:
    import dxcam_cpp as dxcam
except ImportError:
    import dxcam


# ========= LED / LAYOUT CONFIG =========
TOP_LEDS    = 51
RIGHT_LEDS  = 22
BOTTOM_LEDS = 51
LEFT_LEDS   = 23

NUM_LEDS = TOP_LEDS + RIGHT_LEDS + BOTTOM_LEDS + LEFT_LEDS
TOP_BOTTOM_STRIP_HEIGHT = 40
# ======================================


# ========= SERIAL CONFIG =========
COM_PORT = "COM3"
BAUD     = 2_000_000
HEADER   = b"\xAA\x55\xAA\x55"
# ================================


# ========= COLOR / SMOOTHING TUNING =========
BRIGHTNESS       = 1.0
SATURATION_BOOST = 0.9
MIN_VISIBLE      = 10

SMOOTHING_ALPHA  = 0.9
GAMMA            = 1.2
WARM_COOL        = 0.0
# ===========================================


PILLARBOX_MODE = False
FADE_IN_DURATION = 2.0


# ========= PYTHON FADE-OUT =========
FADE_OUT_ENABLED   = True
FADE_OUT_SECONDS   = 1.2
BLACK_FRAME_HOLD_S = 0.15
BLACK_THRESH_SUM   = 0  # 0 = only truly-black triggers fade
# ===================================


@contextlib.contextmanager
def suppress_stderr():
    old_stderr = sys.stderr
    sys.stderr = io.StringIO()
    try:
        yield
    finally:
        sys.stderr = old_stderr


def regions_top_bottom_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    h, w, _ = strip_img.shape
    region_w = w // num_regions
    w_used = region_w * num_regions
    if w_used == 0:
        raise ValueError("region_w became 0; check screen width vs num_regions")
    strip = strip_img[:, :w_used, :].astype(np.float32)
    strip = strip.reshape(h, num_regions, region_w, 3)
    return strip.mean(axis=(0, 2))


def regions_left_right_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    h, w, _ = strip_img.shape
    region_h = h // num_regions
    h_used = region_h * num_regions
    if h_used == 0:
        raise ValueError("region_h became 0; check strip height vs num_regions")
    strip = strip_img[:h_used, :, :].astype(np.float32)
    strip = strip.reshape(num_regions, region_h, w, 3)
    return strip.mean(axis=(1, 2))


def open_serial() -> Optional[serial.Serial]:
    try:
        # write_timeout prevents rare “hang on close” situations on some systems
        ser = serial.Serial(COM_PORT, BAUD, timeout=0, write_timeout=0.2)
        return ser
    except Exception as e:
        print(f"Warning: could not open serial port {COM_PORT}: {e}")
        return None


def serial_worker(ser: Optional[serial.Serial], q: "queue.Queue[np.ndarray]"):
    """
    Worker exits ONLY when it receives sentinel None.
    This prevents shutdown races where stop_evt kills the worker before it sends black frames.
    """
    if ser is None:
        while True:
            item = q.get()
            if item is None:
                break
        return

    try:
        while True:
            led_colors = q.get()
            if led_colors is None:
                break

            frame = HEADER + led_colors.tobytes()
            try:
                ser.write(frame)
                ser.flush()  # ensure it actually leaves the OS buffer
            except Exception as e:
                print(f"Serial write error: {e}")
    finally:
        try:
            if ser is not None and ser.is_open:
                ser.close()
        except Exception:
            pass


def apply_color_correction(led_float: np.ndarray) -> np.ndarray:
    arr = np.clip(led_float, 0, 255).astype(np.float32) / 255.0
    out = np.empty_like(arr)

    for i, (r, g, b) in enumerate(arr):
        h, s, v = colorsys.rgb_to_hsv(r, g, b)

        s = min(1.0, s * SATURATION_BOOST)

        v *= BRIGHTNESS
        if v > 0.0:
            v = v ** GAMMA

        if v < MIN_VISIBLE / 255.0:
            v = 0.0

        r2, g2, b2 = colorsys.hsv_to_rgb(h, s, v)

        if WARM_COOL != 0.0:
            k = 0.15 * WARM_COOL
            r2 = float(np.clip(r2 + k, 0.0, 1.0))
            b2 = float(np.clip(b2 - k, 0.0, 1.0))

        out[i] = (r2, g2, b2)

    return (out * 255.0).astype(np.uint8)


def put_latest(q: "queue.Queue[np.ndarray]", frame: np.ndarray):
    """
    Queue latest frame; if full, drop old and replace.
    This keeps fade smooth even with maxsize=1.
    """
    try:
        q.put_nowait(frame)
    except queue.Full:
        try:
            _ = q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(frame)
        except queue.Full:
            pass


def fade_to_black(q: "queue.Queue[np.ndarray]", last_sent: np.ndarray, duration_s: float, fps: float = 60.0):
    if duration_s <= 0:
        put_latest(q, np.zeros_like(last_sent))
        return

    steps = max(1, int(duration_s * fps))
    base = last_sent.astype(np.float32)

    for i in range(steps + 1):
        t = i / steps
        factor = 1.0 - t
        frame = (base * factor).astype(np.uint8)
        put_latest(q, frame)
        time.sleep(1.0 / fps)


def main():
    with suppress_stderr():
        camera = dxcam.create(output_color="RGB")

    camera.start(target_fps=120, video_mode=True)

    ser = open_serial()
    serial_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)

    worker = threading.Thread(
        target=serial_worker,
        args=(ser, serial_queue),
        daemon=True,
    )
    worker.start()

    screen_w = None
    screen_h = None
    slices_ready = False
    pillar_pixels = 0

    led_float          = np.zeros((NUM_LEDS, 3), dtype=np.float32)
    smoothed_led_float = np.zeros((NUM_LEDS, 3), dtype=np.float32)
    have_smoothed      = False
    led_colors         = np.zeros((NUM_LEDS, 3), dtype=np.uint8)

    start_time = time.perf_counter()

    # Rate limit (USB sanity)
    MAX_SEND_FPS = 60.0
    min_send_dt  = 1.0 / MAX_SEND_FPS
    last_send_t  = 0.0

    # Fade-out state
    last_sent    = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
    black_since  = None
    fade_mode    = False
    fade_start_t = 0.0
    fade_base    = np.zeros((NUM_LEDS, 3), dtype=np.float32)

    try:
        while True:
            frame = camera.get_latest_frame()
            if frame is None:
                continue

            if not slices_ready:
                h, w, _ = frame.shape
                screen_w, screen_h = w, h

                if PILLARBOX_MODE:
                    active_width_16_9 = int(screen_h * 16 / 9)
                    pillar_pixels = (screen_w - active_width_16_9) // 2 if active_width_16_9 < screen_w else 0
                else:
                    pillar_pixels = 0

                if pillar_pixels * 2 > screen_w:
                    pillar_pixels = 0

                slices_ready = True

            strip_h = TOP_BOTTOM_STRIP_HEIGHT

            if pillar_pixels > 0:
                active_left  = pillar_pixels
                active_right = screen_w - pillar_pixels
                active_width = active_right - active_left
            else:
                active_left  = 0
                active_right = screen_w
                active_width = screen_w

            top_img    = frame[0:strip_h, active_left:active_right, :]
            bottom_img = frame[screen_h - strip_h:screen_h, active_left:active_right, :]

            top_region_width = active_width / TOP_LEDS
            side_w = int(top_region_width)
            if side_w <= 0:
                side_w = 1
            if active_width < 2 * side_w:
                side_w = max(1, active_width // 4)

            left_x_start  = max(0, active_left)
            left_x_end    = min(screen_w, left_x_start + side_w)
            right_x_end   = min(screen_w, active_right)
            right_x_start = max(0, right_x_end - side_w)

            v_top    = strip_h
            v_bottom = screen_h - strip_h

            left_img  = frame[v_top:v_bottom, left_x_start:left_x_end, :]
            right_img = frame[v_top:v_bottom, right_x_start:right_x_end, :]

            top_means    = regions_top_bottom_vec(top_img, TOP_LEDS)
            bottom_means = regions_top_bottom_vec(bottom_img, BOTTOM_LEDS)
            right_means  = regions_left_right_vec(right_img, RIGHT_LEDS)
            left_means   = regions_left_right_vec(left_img, LEFT_LEDS)

            idx = 0
            led_float[idx:idx + TOP_LEDS] = top_means
            idx += TOP_LEDS
            led_float[idx:idx + RIGHT_LEDS] = right_means
            idx += RIGHT_LEDS
            led_float[idx:idx + BOTTOM_LEDS] = bottom_means[::-1]
            idx += BOTTOM_LEDS
            led_float[idx:idx + LEFT_LEDS] = left_means[::-1]

            if not have_smoothed:
                smoothed_led_float[:] = led_float
                have_smoothed = True
            else:
                smoothed_led_float[:] = (
                    SMOOTHING_ALPHA * smoothed_led_float
                    + (1.0 - SMOOTHING_ALPHA) * led_float
                )

            led_colors[:] = apply_color_correction(smoothed_led_float)

            elapsed = time.perf_counter() - start_time
            if elapsed < FADE_IN_DURATION:
                t = elapsed / FADE_IN_DURATION
                fade_in_factor = t * t
            else:
                fade_in_factor = 1.0

            if fade_in_factor < 1.0:
                to_send = (led_colors.astype(np.float32) * fade_in_factor).astype(np.uint8)
            else:
                to_send = led_colors

            now_t = time.perf_counter()

            frame_sum = int(to_send.sum())
            is_black = frame_sum <= BLACK_THRESH_SUM

            if FADE_OUT_ENABLED:
                if is_black:
                    if black_since is None:
                        black_since = now_t
                    if (not fade_mode) and (now_t - black_since >= BLACK_FRAME_HOLD_S):
                        fade_mode = True
                        fade_start_t = now_t
                        fade_base = last_sent.astype(np.float32)
                else:
                    black_since = None
                    fade_mode = False

            if fade_mode:
                t = (now_t - fade_start_t) / max(FADE_OUT_SECONDS, 1e-6)
                if t >= 1.0:
                    send_frame = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
                else:
                    factor = 1.0 - t
                    send_frame = (fade_base * factor).astype(np.uint8)
            else:
                send_frame = to_send

            # IMPORTANT: keep sending black even after fade (heartbeat),
            # so any rare corruption gets overwritten quickly.
            if (now_t - last_send_t) >= min_send_dt:
                last_send_t = now_t
                last_sent = send_frame.copy()
                put_latest(serial_queue, last_sent)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        camera.stop()

        # Fade to black on exit, then force multiple black frames
        if FADE_OUT_ENABLED:
            fade_to_black(serial_queue, last_sent, duration_s=FADE_OUT_SECONDS, fps=60.0)

        off = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
        for _ in range(8):                 # send a few times to be extra sure
            put_latest(serial_queue, off)
            time.sleep(0.01)

        # Stop worker cleanly AFTER off frames are queued
        serial_queue.put(None)
        worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
