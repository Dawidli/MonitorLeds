import dxcam
import time
import contextlib
import sys
import io
import gc


@contextlib.contextmanager
def suppress_stderr():
    """Temporarily silence stderr (hides comtypes 'Exception ignored' spam)."""
    old_stderr = sys.stderr
    sys.stderr = io.StringIO()
    try:
        yield
    finally:
        sys.stderr = old_stderr


def test_region_grab():
    """Test grabbing only a small top strip of the screen."""
    camera = dxcam.create(output_color="RGB")
    region = (0, 0, 3440, 80)

    frames = 200
    start = time.perf_counter()

    for _ in range(frames):
        _ = camera.grab(region=region)

    duration = time.perf_counter() - start
    fps = frames / duration
    print(f"[REGION] {region}, frames={frames}, time={duration:.3f}s, FPS: {fps:.1f}")

    del camera
    gc.collect()


def test_fullscreen_grab(duration_sec: float = 10.0):
    """Measure fullscreen grab FPS over a fixed duration."""
    camera = dxcam.create(output_color="RGB")
    region = None  # None = fullscreen

    start = time.perf_counter()
    end_time = start + duration_sec
    frames = 0

    while time.perf_counter() < end_time:
        _ = camera.grab(region=region)
        frames += 1

    duration = time.perf_counter() - start
    fps = frames / duration
    print(f"[FULLSCREEN] duration={duration:.3f}s, frames={frames}, FPS: {fps:.1f}")

    del camera
    gc.collect()


if __name__ == "__main__":
    with suppress_stderr():
        # Pick which test you want to run:
        # test_region_grab()
        test_fullscreen_grab(duration_sec=10.0)
