# stream_benchmark.py — run directly in PyCharm
# Edit the CONFIG block only.

import time
import numpy as np
import serial

# ========= CONFIG (edit these) =========
COM_PORT   = "COM3"       # e.g. "COM3"
BAUD       = 2_000_000    # try 2_000_000 after it's stable
NUM_LEDS   = 147
FPS_TARGET = 144          # target; we’ll print measured FPS
# ======================================


def make_frame(t, n=NUM_LEDS):
    """Return GRB uint8 array (n,3). Fast moving rainbow."""
    # generate 0..255 index per LED, shifted by time
    idx = (np.arange(n, dtype=np.int32) + int(t * 60)) & 255

    # vectorized wheel (RGB)
    k = idx.astype(np.uint8)
    r = np.where(k < 85, 255 - k*3,
        np.where(k < 170, 0, (k-170)*3))
    g = np.where(k < 85, k*3,
        np.where(k < 170, 255 - (k-85)*3, 0))
    b = np.where(k < 85, 0,
        np.where(k < 170, (k-85)*3, 255 - (k-170)*3))
    rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)

    # WS2812B wants GRB
    return rgb[:, [1, 0, 2]]


def wait_for_ready(ser, timeout=3.0):
    """Consume lines until READY or timeout; non-fatal if not seen."""
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"[serial] {line}")
            if line == "READY":
                return True
    return False


def main():
    ser = serial.Serial(COM_PORT, BAUD, timeout=0)
    try:
        # brief pause; some boards reset on port open
        time.sleep(0.2)
        wait_for_ready(ser)

        per = 1.0 / FPS_TARGET
        next_t = time.perf_counter()
        t0 = time.perf_counter()
        frames = 0
        last_report = t0

        while True:
            t = time.perf_counter() - t0
            grb = make_frame(t)  # (NUM_LEDS,3) uint8

            # send header + payload
            ser.write(b"\xAA\x55")
            ser.write(grb.tobytes())

            frames += 1
            now = time.perf_counter()
            if now - last_report >= 1.0:
                fps = frames / (now - last_report)
                kbps = (2 + NUM_LEDS*3) * fps / 1024.0  # rough wire rate
                print(f"FPS={fps:6.1f}   serial≈{kbps:6.1f} KiB/s   BAUD={BAUD}")
                frames = 0
                last_report = now

            # pace to target FPS
            next_t += per
            sleep_t = next_t - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                # fell behind; resync (prevents spiral of death)
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("Closed.")


if __name__ == "__main__":
    main()
