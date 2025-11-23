import os
import dxcam
import numpy as np
from PIL import Image  # pip install pillow


# ===== CONFIG =====
TOP_LEDS    = 51
RIGHT_LEDS  = 22
BOTTOM_LEDS = 51
LEFT_LEDS   = 23

EDGE_THICKNESS = 80          # thickness of top/bottom and corner zone (pixels)
OUTPUT_DIR     = "captures"  # folder for all JPEGs
# ==================


def avg_horizontal_band(band: np.ndarray, num_leds: int) -> np.ndarray:
    """
    band: (thickness, width, 3)
    Split along width into num_leds regions and average each.
    Returns (num_leds, 3) uint8.
    """
    thickness, w, _ = band.shape
    pixels_per_led = w / num_leds
    colors = np.zeros((num_leds, 3), dtype=np.float32)

    for i in range(num_leds):
        x0 = int(i * pixels_per_led)
        x1 = int((i + 1) * pixels_per_led)
        if x1 <= x0:
            x1 = x0 + 1
        segment = band[:, x0:x1, :]
        colors[i] = segment.mean(axis=(0, 1))

    return np.clip(colors, 0, 255).astype(np.uint8)


def avg_vertical_band(band: np.ndarray, num_leds: int) -> np.ndarray:
    """
    band: (height, thickness, 3)
    Split along height into num_leds regions and average each.
    Returns (num_leds, 3) uint8.
    """
    h, thickness, _ = band.shape
    pixels_per_led = h / num_leds
    colors = np.zeros((num_leds, 3), dtype=np.float32)

    for i in range(num_leds):
        y0 = int(i * pixels_per_led)
        y1 = int((i + 1) * pixels_per_led)
        if y1 <= y0:
            y1 = y0 + 1
        segment = band[y0:y1, :, :]
        colors[i] = segment.mean(axis=(0, 1))

    return np.clip(colors, 0, 255).astype(np.uint8)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    camera = dxcam.create(output_color="RGB")
    frame = camera.grab()

    if frame is None:
        print("Failed to grab frame. Is the main monitor active?")
        return

    h, w, _ = frame.shape
    print(f"Screen resolution: {w} x {h}")

    # Corner rule:
    # - Top owns y=0 .. EDGE_THICKNESS
    # - Bottom owns y=h-EDGE_THICKNESS .. h
    # - Left/Right own only y=EDGE_THICKNESS .. h-EDGE_THICKNESS
    y_mid_start = EDGE_THICKNESS
    y_mid_end   = h - EDGE_THICKNESS
    mid_h       = y_mid_end - y_mid_start

    # ===== 1) Extract raw edge captures =====
    top_band    = frame[0:EDGE_THICKNESS, :, :]
    bottom_band = frame[h-EDGE_THICKNESS:h, :, :]
    left_band   = frame[y_mid_start:y_mid_end, 0:EDGE_THICKNESS, :]
    right_band  = frame[y_mid_start:y_mid_end, w-EDGE_THICKNESS:w, :]

    # ===== 2) Build averaged-region images for each edge =====

    # --- Top (horizontal, full width) ---
    top_colors = avg_horizontal_band(top_band, TOP_LEDS)
    top_regions = np.zeros_like(top_band)
    pixels_per_led = w / TOP_LEDS
    for i in range(TOP_LEDS):
        x0 = int(i * pixels_per_led)
        x1 = int((i + 1) * pixels_per_led)
        if x1 <= x0:
            x1 = x0 + 1
        top_regions[:, x0:x1, :] = top_colors[i]

    # --- Bottom (horizontal, full width) ---
    bottom_colors = avg_horizontal_band(bottom_band, BOTTOM_LEDS)
    bottom_regions = np.zeros_like(bottom_band)
    pixels_per_led = w / BOTTOM_LEDS
    for i in range(BOTTOM_LEDS):
        x0 = int(i * pixels_per_led)
        x1 = int((i + 1) * pixels_per_led)
        if x1 <= x0:
            x1 = x0 + 1
        bottom_regions[:, x0:x1, :] = bottom_colors[i]

    # --- Left (vertical, middle only) ---
    left_colors = avg_vertical_band(left_band, LEFT_LEDS)
    left_regions = np.zeros_like(left_band)
    pixels_per_led = mid_h / LEFT_LEDS
    for i in range(LEFT_LEDS):
        y0 = int(i * pixels_per_led)
        y1 = int((i + 1) * pixels_per_led)
        if y1 <= y0:
            y1 = y0 + 1
        left_regions[y0:y1, :, :] = left_colors[i]

    # --- Right (vertical, middle only) ---
    right_colors = avg_vertical_band(right_band, RIGHT_LEDS)
    right_regions = np.zeros_like(right_band)
    pixels_per_led = mid_h / RIGHT_LEDS
    for i in range(RIGHT_LEDS):
        y0 = int(i * pixels_per_led)
        y1 = int((i + 1) * pixels_per_led)
        if y1 <= y0:
            y1 = y0 + 1
        right_regions[y0:y1, :, :] = right_colors[i]

    # ===== 3) Per-edge compare JPEGs (capture + averages) =====

    # Top & bottom: wider than high, stack vertically
    top_compare = np.vstack([top_band, top_regions])
    Image.fromarray(top_compare).save(
        os.path.join(OUTPUT_DIR, "top_compare.jpg"), "JPEG"
    )

    bottom_compare = np.vstack([bottom_band, bottom_regions])
    Image.fromarray(bottom_compare).save(
        os.path.join(OUTPUT_DIR, "bottom_compare.jpg"), "JPEG"
    )

    # Left & right: higher than wide -> put capture + averages side-by-side

    # LEFT: left = real capture, right = averages (middle band only)
    left_compare_array = np.zeros(
        (mid_h, EDGE_THICKNESS * 2, 3), dtype=np.uint8
    )
    left_compare_array[:, 0:EDGE_THICKNESS, :] = left_band
    left_compare_array[:, EDGE_THICKNESS:EDGE_THICKNESS * 2, :] = left_regions
    Image.fromarray(left_compare_array).save(
        os.path.join(OUTPUT_DIR, "left_compare.jpg"), "JPEG"
    )

    # RIGHT: left = averages, right = real capture (middle band only)
    right_compare_array = np.zeros(
        (mid_h, EDGE_THICKNESS * 2, 3), dtype=np.uint8
    )
    right_compare_array[:, 0:EDGE_THICKNESS, :] = right_regions
    right_compare_array[:, EDGE_THICKNESS:EDGE_THICKNESS * 2, :] = right_band
    Image.fromarray(right_compare_array).save(
        os.path.join(OUTPUT_DIR, "right_compare.jpg"), "JPEG"
    )

    print("Saved: top_compare, bottom_compare, left_compare, right_compare")

    # ===== 4) Build full-frame combined images (with black center) =====
    # Canvas is same size as the original screen
    canvas_h = h
    canvas_w = w

    # We keep top/bottom full width.
    # For left/right we only change their VISIBLE width in the combined images.
    visual_lr_width = int(round(w / TOP_LEDS))  # ~67 px for 3440/51

    # --- Combined real captures ---
    combined_capture = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    # top edge: full width
    combined_capture[0:EDGE_THICKNESS, 0:w, :] = top_band
    # bottom edge: full width
    combined_capture[h - EDGE_THICKNESS:h, 0:w, :] = bottom_band

    # left edge: resize from (mid_h, EDGE_THICKNESS) -> (mid_h, visual_lr_width)
    left_cap_vis = np.array(
        Image.fromarray(left_band).resize(
            (visual_lr_width, mid_h), Image.BILINEAR
        )
    )
    combined_capture[y_mid_start:y_mid_end, 0:visual_lr_width, :] = left_cap_vis

    # right edge: same idea, placed on the right side
    right_cap_vis = np.array(
        Image.fromarray(right_band).resize(
            (visual_lr_width, mid_h), Image.BILINEAR
        )
    )
    combined_capture[y_mid_start:y_mid_end,
    w - visual_lr_width:w, :] = right_cap_vis

    Image.fromarray(combined_capture).save(
        os.path.join(OUTPUT_DIR, "all_edges_capture.jpg"), "JPEG"
    )

    # --- Combined averaged regions ---
    combined_regions = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    combined_regions[0:EDGE_THICKNESS, 0:w, :] = top_regions
    combined_regions[h - EDGE_THICKNESS:h, 0:w, :] = bottom_regions

    left_reg_vis = np.array(
        Image.fromarray(left_regions).resize(
            (visual_lr_width, mid_h), Image.BILINEAR
        )
    )
    combined_regions[y_mid_start:y_mid_end,
    0:visual_lr_width, :] = left_reg_vis

    right_reg_vis = np.array(
        Image.fromarray(right_regions).resize(
            (visual_lr_width, mid_h), Image.BILINEAR
        )
    )
    combined_regions[y_mid_start:y_mid_end,
    w - visual_lr_width:w, :] = right_reg_vis

    Image.fromarray(combined_regions).save(
        os.path.join(OUTPUT_DIR, "all_edges_regions.jpg"), "JPEG"
    )

    print("Saved: all_edges_capture.jpg, all_edges_regions.jpg (left/right visually narrowed).")


if __name__ == "__main__":
    main()
