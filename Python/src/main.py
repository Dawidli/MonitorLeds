import time
import serial

COM_PORT       = "COM3"
BAUD           = 115200


# ------- Config --------
BRIGHTNESS     = 100  # 0..255 overall brightness scaling
MIN_VISIBLE    = 10   # any channel < 10 becomes 0
# -----------------------


def apply_rules(r, g, b, brightness=BRIGHTNESS):
    """Clamp 0..255, scale by brightness, and zero-out near-black values."""
    # Clamp inputs first
    r = max(0, min(255, int(r)))
    g = max(0, min(255, int(g)))
    b = max(0, min(255, int(b)))

    # Apply MIN_VISIBLE rule
    r = 0 if r < MIN_VISIBLE else r
    g = 0 if g < MIN_VISIBLE else g
    b = 0 if b < MIN_VISIBLE else b

    # Scale by global brightness (integer math, 0..255)
    if brightness != 255:
        r = (r * brightness) // 255
        g = (g * brightness) // 255
        b = (b * brightness) // 255



    return r, g, b

def send_color(ser, r, g, b):
    r, g, b = apply_rules(r, g, b)
    ser.write(
        f"{r},{g},{b}\n".encode("ascii"))

if __name__ == "__main__":
    with serial.Serial(COM_PORT, BAUD, timeout=1) as ser:
        print("Connected to Arduino.")
        time.sleep(2.0)  # allow Arduino to reset after opening serial

        # Examples / quick test cycle
        send_color(ser, 0, 0, 11);       time.sleep(0.5)
