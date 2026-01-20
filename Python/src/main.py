import time
import sys
import io
import contextlib
import threading
import queue
import os
from typing import Optional

import numpy as np
import serial

try:
    import dxcam_cpp as dxcam
except ImportError:
    import dxcam


# ========= LOGGING =========
sys.stderr = open(r"C:\Users\dawid\PycharmProjects\MonitorLeds\led_error.log", "a", buffering=1)
sys.stdout = sys.stderr
# ===========================


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


# ========= SMOOTHING TUNING =========
SMOOTHING_ALPHA = 0.9
# ====================================


# ========= STARTUP FADE-IN =========
FADE_IN_DURATION = 2.0
# ===================================


# ========= EXIT FADE-OUT ONLY =========
EXIT_FADE_SECONDS = 1.2
# =====================================


# ========= STOP FLAG (lock event) =========
STOP_FLAG_PATH = r"C:\Users\dawid\PycharmProjects\MonitorLeds\stop.flag"
# =========================================


@contextlib.contextmanager
def suppress_stderr():
    old_stderr = sys.stderr
    sys.stderr = io.StringIO()
    try:
        yield
    finally:
        sys.stderr = old_stderr


def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def regions_top_bottom_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    h, w, _ = strip_img.shape
    region_w = w // num_regions
    w_used = region_w * num_regions
    if w_used == 0:
        raise ValueError("region_w became 0; check screen width vs num_regions")
    strip = strip_img[:, :w_used, :].astype(np.float32).reshape(h, num_regions, region_w, 3)
    return strip.mean(axis=(0, 2))


def regions_left_right_vec(strip_img: np.ndarray, num_regions: int) -> np.ndarray:
    h, w, _ = strip_img.shape
    region_h = h // num_regions
    h_used = region_h * num_regions
    if h_used == 0:
        raise ValueError("region_h became 0; check strip height vs num_regions")
    strip = strip_img[:h_used, :, :].astype(np.float32).reshape(num_regions, region_h, w, 3)
    return strip.mean(axis=(1, 2))


def open_serial() -> Optional[serial.Serial]:
    try:
        ser = serial.Serial(
            COM_PORT,
            BAUD,
            timeout=0,
            write_timeout=None
        )
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser
    except Exception as e:
        print(f"Warning: could not open serial port {COM_PORT}: {e}")
        return None


def write_all(ser: serial.Serial, data: bytes) -> None:
    mv = memoryview(data)
    sent = 0
    while sent < len(mv):
        n = ser.write(mv[sent:])
        if n is None:
            n = 0
        if n == 0:
            time.sleep(0.001)
            continue
        sent += n


def serial_worker(ser: Optional[serial.Serial], q: "queue.Queue[bytes]"):
    if ser is None:
        while True:
            item = q.get()
            if item is None:
                break
        return

    try:
        while True:
            frame_bytes = q.get()
            if frame_bytes is None:
                break
            try:
                write_all(ser, frame_bytes)
            except Exception as e:
                print(f"Serial write error: {e}")
    finally:
        try:
            if ser is not None and ser.is_open:
                ser.close()
        except Exception:
            pass


def put_latest(q: "queue.Queue[bytes]", frame_bytes: bytes):
    try:
        q.put_nowait(frame_bytes)
    except queue.Full:
        try:
            _ = q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(frame_bytes)
        except queue.Full:
            pass


def stop_flag_is_set() -> bool:
    try:
        return os.path.exists(STOP_FLAG_PATH)
    except Exception:
        return False


def clear_stop_flag():
    try:
        if os.path.exists(STOP_FLAG_PATH):
            os.remove(STOP_FLAG_PATH)
    except Exception:
        pass


def build_packet(payload: bytes, seq: int) -> bytes:
    ln = len(payload)
    meta = bytes([seq & 0xFF]) + ln.to_bytes(2, "little")
    crc = crc16_ccitt(meta + payload).to_bytes(2, "little")
    return HEADER + meta + payload + crc


def send_frame(serial_queue: "queue.Queue[bytes]", frame_u8: np.ndarray, seq: int) -> int:
    pkt = build_packet(frame_u8.tobytes(), seq)
    put_latest(serial_queue, pkt)
    return (seq + 1) & 0xFF


def fade_to_black(serial_queue: "queue.Queue[bytes]", last_sent: np.ndarray, seconds: float, seq: int, fps: float = 60.0) -> int:
    steps = max(1, int(seconds * fps))
    base = last_sent.astype(np.float32)

    for i in range(steps + 1):
        t = i / steps
        factor = 1.0 - t
        frame = (base * factor).astype(np.uint8)
        seq = send_frame(serial_queue, frame, seq)
        time.sleep(1.0 / fps)

    return seq


def main():
    clear_stop_flag()

    with suppress_stderr():
        camera = dxcam.create(output_color="RGB")

    # Reduce capture pressure/jitter
    camera.start(target_fps=60, video_mode=True)

    ser = open_serial()
    serial_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=1)
    worker = threading.Thread(target=serial_worker, args=(ser, serial_queue), daemon=True)
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

    # Send cap (this matters)
    MAX_SEND_FPS = 45.0
    min_send_dt  = 1.0 / MAX_SEND_FPS
    last_send_t  = 0.0

    last_sent = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
    seq = 0

    try:
        while True:
            if stop_flag_is_set():
                clear_stop_flag()
                break

            frame = camera.get_latest_frame()
            if frame is None:
                continue

            if not slices_ready:
                h, w, _ = frame.shape
                screen_w, screen_h = w, h
                pillar_pixels = 0
                slices_ready = True

            strip_h = TOP_BOTTOM_STRIP_HEIGHT

            active_left  = pillar_pixels
            active_right = screen_w - pillar_pixels
            active_width = active_right - active_left

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
            led_float[idx:idx + TOP_LEDS] = top_means; idx += TOP_LEDS
            led_float[idx:idx + RIGHT_LEDS] = right_means; idx += RIGHT_LEDS
            led_float[idx:idx + BOTTOM_LEDS] = bottom_means[::-1]; idx += BOTTOM_LEDS
            led_float[idx:idx + LEFT_LEDS] = left_means[::-1]

            if not have_smoothed:
                smoothed_led_float[:] = led_float
                have_smoothed = True
            else:
                smoothed_led_float[:] = (
                    SMOOTHING_ALPHA * smoothed_led_float
                    + (1.0 - SMOOTHING_ALPHA) * led_float
                )

            led_colors[:] = np.clip(smoothed_led_float, 0, 255).astype(np.uint8)

            # Fade-in (startup only)
            elapsed = time.perf_counter() - start_time
            if elapsed < FADE_IN_DURATION:
                t = elapsed / FADE_IN_DURATION
                fade_in_factor = t * t
                to_send = (led_colors.astype(np.float32) * fade_in_factor).astype(np.uint8)
            else:
                to_send = led_colors

            now_t = time.perf_counter()
            if (now_t - last_send_t) >= min_send_dt:
                last_send_t = now_t
                last_sent = to_send.copy()
                seq = send_frame(serial_queue, last_sent, seq)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            camera.stop()
        except Exception:
            pass

        # Exit fade only (clean + deterministic)
        seq = fade_to_black(serial_queue, last_sent, EXIT_FADE_SECONDS, seq, fps=60.0)

        off = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
        for _ in range(8):
            seq = send_frame(serial_queue, off, seq)
            time.sleep(0.01)

        serial_queue.put(None)
        worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
