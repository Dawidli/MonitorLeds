import time
import serial

COM_PORT = "COM3"
BAUD = 1_000_000
NUM_LEDS = 147
DELAY = 0.1  # seconds between moves


def wait_for_ready(ser, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"[serial] {line}")
        if line == "READY":
            return True
    return False


def main():
    ser = serial.Serial(COM_PORT, BAUD, timeout=0.5)
    time.sleep(0.2)
    wait_for_ready(ser)

    try:
        while True:
            for i in range(NUM_LEDS):
                cmd = f"PIX {i} 0 0 255\n".encode()
                ser.write(cmd)
                reply = ser.readline().decode(errors="ignore").strip()
                if reply:
                    print(f"LED {i}: {reply}")
                time.sleep(DELAY)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("Closed.")


if __name__ == "__main__":
    main()
