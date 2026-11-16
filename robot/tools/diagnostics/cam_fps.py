"""Measure raw camera fps with auto vs manual exposure (no display, no processing)."""
import sys, time
import cv2
dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video2"
for label, exp in (("auto", None), ("manual 15ms", 150), ("manual 8ms", 80)):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720); cap.set(cv2.CAP_PROP_FPS, 30)
    if exp is None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    else:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); cap.set(cv2.CAP_PROP_EXPOSURE, exp)
    for _ in range(10):
        cap.read()
    n, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < 3:
        n += cap.read()[0]
    ok, fr = cap.read()
    print(f"{label:12s}: {n / (time.perf_counter() - t0):5.1f} fps, mean brightness {fr.mean():.0f}")
    cap.release()
