# ultra-simple ambilight (explicit edge order, no rev flags)

import time
import numpy as np
import serial
import dxcam

# ====== CONFIG (edit these) ======
COM_PORT     = "COM3"
BAUD         = 2_000_000
NUM_LEDS     = 147

# LED counts per edge (must sum to NUM_LEDS)
TOP, RIGHT, BOTTOM, LEFT = 51, 22, 51, 23
assert TOP + RIGHT + BOTTOM + LEFT == NUM_LEDS

# >>> Edge order around the screen (clockwise around your monitor).
# If top/bottom are swapped for you, just swap them here.
# Examples:
#   ["top","right","bottom","left"]     # typical clockwise from top-left
#   ["bottom","right","top","left"]     # swap top and bottom
EDGE_ORDER = ["bottom","right","top","left"]  # <-- set this how you want

FPS_TARGET     = 144
EDGE_THICKNESS = 36       # pixels sampled at the border
DOWNSCALE      = 3        # 2–4 is fine; bigger = faster
BRIGHTNESS_MAX = 30       # cap to avoid washout (0..255)
GAMMA_ENCODE   = 2.0      # perceptual dimming; 0 to disable
CALIBRATE      = False     # show solid edge colors in EDGE_ORDER
OUTPUT_IDX     = 0        # which monitor to capture (0 = primary)
# ================================


def wait_for_ready(ser, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("[serial]", line)
            if line == "READY":
                return True
    return False


def build_rects(w, h, thickness):
    """Return dict of edge->list of (x0,x1,y0,y1) rectangles in the
       natural on-screen direction for a clockwise perimeter:
       top:    right -> left
       right:  bottom -> top
       bottom: left  -> right
       left:   top   -> bottom
    """
    rects = {}

    # horizontal strips
    def h_segments(count, x0, x1, y0, y1):
        segs = []
        for i in range(count):
            xa = int(x0 + (x1 - x0) * i / count)
            xb = int(x0 + (x1 - x0) * (i + 1) / count)
            segs.append((xa, xb, y0, y1))
        return segs

    # vertical strips
    def v_segments(count, x0, x1, y0, y1):
        segs = []
        for i in range(count):
            ya = int(y0 + (y1 - y0) * i / count)
            yb = int(y0 + (y1 - y0) * (i + 1) / count)
            segs.append((x0, x1, ya, yb))
        return segs

    # bottom: left -> right
    rects["bottom"] = h_segments(BOTTOM, 0, w, h - thickness, h)
    # right: bottom -> top
    rects["right"]  = v_segments(RIGHT,  w - thickness, w, 0, h)[::-1]
    # top: right -> left
    rects["top"]    = h_segments(TOP, 0, w, 0, thickness)[::-1]
    # left: top -> bottom
    rects["left"]   = v_segments(LEFT, 0, thickness, 0, h)

    return rects


def show_calibration(ser, edge_order, counts):
    """Bottom=RED, Right=GREEN, Top=BLUE, Left=YELLOW in the chosen EDGE_ORDER."""
    color_grb = {
        "bottom": (0, 255, 0),      # red
        "right":  (255, 0, 0),      # green
        "top":    (0, 0, 255),      # blue
        "left":   (255, 255, 0),    # yellow
    }
    frame = np.zeros((NUM_LEDS, 3), dtype=np.uint8)
    idx = 0
    for edge in edge_order:
        n = counts[edge]
        frame[idx:idx+n] = color_grb[edge]
        idx += n

    t_end = time.perf_counter() + 3.0
    while time.perf_counter() < t_end:
        ser.write(b"\xAA\x55"); ser.write(frame.tobytes())
        time.sleep(1/144)


def main():
    # serial
    ser = serial.Serial(COM_PORT, BAUD, timeout=0)
    time.sleep(0.2); wait_for_ready(ser)

    # capture
    cam = dxcam.create(output_idx=OUTPUT_IDX)
    first = cam.grab()
    if first is None:
        ser.close()
        raise RuntimeError("dxcam.grab() returned None. Is the display active?")
    H, W = first.shape[:2]
    W2, H2 = max(1, W // DOWNSCALE), max(1, H // DOWNSCALE)
    thickness = max(2, min(EDGE_THICKNESS, W2 // 3, H2 // 3))
    print(f"[info] capture={W}x{H}  downscaled={W2}x{H2}  thickness={thickness}  order={EDGE_ORDER}")

    cam.start(target_fps=FPS_TARGET, video_mode=True)

    # build rectangles for each edge
    per_edge = build_rects(W2, H2, thickness)
    counts = {"top": TOP, "right": RIGHT, "bottom": BOTTOM, "left": LEFT}

    # build the final LED sequence according to EDGE_ORDER
    rects = []
    for edge in EDGE_ORDER:
        rects.extend(per_edge[edge])
    rects = rects[:NUM_LEDS]

    if CALIBRATE:
        show_calibration(ser, EDGE_ORDER, counts)
        cam.stop(); ser.close(); print("Closed (calibration)."); return

    # gamma LUT (encode)
    lut = None
    if GAMMA_ENCODE and GAMMA_ENCODE > 0:
        x = np.linspace(0, 1, 256)
        lut = (np.clip(x ** GAMMA_ENCODE, 0, 1) * 255.0 + 0.5).astype(np.uint8)

    per = 1.0 / FPS_TARGET
    next_t = time.perf_counter()
    frames, last = 0, time.perf_counter()

    try:
        while True:
            img = cam.get_latest_frame()   # RGB
            if img is None:
                continue
            if (img.shape[1], img.shape[0]) != (W2, H2):
                img = img[::DOWNSCALE, ::DOWNSCALE]

            out = np.empty((NUM_LEDS, 3), dtype=np.uint8)  # GRB
            for i, (x0, x1, y0, y1) in enumerate(rects):
                if x1 <= x0 or y1 <= y0:
                    out[i] = (0, 0, 0); continue
                roi = img[y0:y1, x0:x1]           # RGB
                if roi.size == 0:
                    out[i] = (0, 0, 0); continue
                r, g, b = roi.reshape(-1, 3).mean(axis=0)

                # brightness cap
                if BRIGHTNESS_MAX < 255:
                    s = BRIGHTNESS_MAX / 255.0
                    r *= s; g *= s; b *= s

                if lut is not None:
                    out[i] = (lut[int(g)], lut[int(r)], lut[int(b)])
                else:
                    out[i] = (int(g), int(r), int(b))

            ser.write(b"\xAA\x55"); ser.write(out.tobytes())

            frames += 1
            now = time.perf_counter()
            if now - last >= 1.0:
                print(f"FPS ~ {frames/(now-last):5.1f}")
                frames, last = 0, now

            next_t += per
            dt = next_t - time.perf_counter()
            if dt > 0: time.sleep(dt)
            else: next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        try: cam.stop()
        except Exception: pass
        try: ser.close()
        except Exception: pass
        print("Closed.")


if __name__ == "__main__":
    main()
