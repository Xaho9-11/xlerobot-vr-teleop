"""Probe USB cameras (skips the laptop's integrated one): negotiated modes, real fps, one sample JPEG each.
Usage: cam_probe.py [device ...]   e.g. cam_probe.py /dev/video2"""
import glob, os, sys, time
import cv2

def name(dev):
    try:
        return open(f"/sys/class/video4linux/{os.path.basename(dev)}/name").read().strip()
    except OSError:
        return "?"

devs = sys.argv[1:] or [d for d in sorted(glob.glob("/dev/video*")) if "Integrated" not in name(d)]
MODES = [("MJPG", 640, 480, 30), ("MJPG", 1280, 720, 30), ("MJPG", 1920, 1080, 30), ("YUYV", 640, 480, 30)]
os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
if not devs:
    print("no external camera found"); sys.exit(1)
for dev in devs:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"{dev} ({name(dev)}): cannot open (metadata node?)"); continue
    print(f"== {dev}  {name(dev)}")
    for fourcc, w, h, fps in MODES:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h); cap.set(cv2.CAP_PROP_FPS, fps)
        got_w, got_h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cc = int(cap.get(cv2.CAP_PROP_FOURCC)); got_cc = "".join(chr((cc >> 8 * i) & 0xFF) for i in range(4))
        for _ in range(5):
            cap.read()
        n, t0, frame = 0, time.perf_counter(), None
        while time.perf_counter() - t0 < 2.0:
            ok, fr = cap.read()
            if ok:
                n += 1; frame = fr
        real = n / (time.perf_counter() - t0)
        print(f"  asked {fourcc} {w}x{h}@{fps} -> got {got_cc} {got_w}x{got_h}, measured {real:5.1f} fps")
        if frame is not None and fourcc == "MJPG" and w == 1280:
            out = os.path.expanduser(f"~/fmp_logs/cam_sample_{os.path.basename(dev)}.jpg")
            cv2.imwrite(out, frame); print(f"  sample frame: {out}")
    cap.release()
