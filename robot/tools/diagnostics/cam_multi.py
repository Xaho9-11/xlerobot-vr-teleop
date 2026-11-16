"""Open several USB cameras at once and measure each one's fps (checks shared USB 2.0 bandwidth).
Prints which USB port path each /dev/videoN sits on.
Usage: cam_multi.py WIDTH HEIGHT [dev[:WxH] ...]   (per-device size overrides the default)"""
import glob, os, sys, threading, time
import cv2

w, h = int(sys.argv[1]), int(sys.argv[2])
devs = sys.argv[3:] or [d for d in sorted(glob.glob("/dev/video*"))
                        if "Integrated" not in open(f"/sys/class/video4linux/{os.path.basename(d)}/name").read()]

def usb_path(dev):
    p = os.path.realpath(f"/sys/class/video4linux/{os.path.basename(dev)}/device")
    return os.path.basename(os.path.dirname(p))  # e.g. 1-2.1.4  (bus-port.port...)

caps, sizes = {}, {}
for spec in devs:
    d, _, sz = spec.partition(":")
    dw, dh = (int(v) for v in sz.split("x")) if sz else (w, h)
    c = cv2.VideoCapture(d, cv2.CAP_V4L2)
    if not c.isOpened():
        continue  # metadata nodes
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, dw); c.set(cv2.CAP_PROP_FRAME_HEIGHT, dh); c.set(cv2.CAP_PROP_FPS, 30)
    caps[d], sizes[d] = c, f"{dw}x{dh}"
counts = {d: 0 for d in caps}; fails = {d: 0 for d in caps}; stop = threading.Event()

def run(d):
    while not stop.is_set():
        ok, _ = caps[d].read()
        if ok: counts[d] += 1
        else: fails[d] += 1; time.sleep(0.01)

for d in caps:
    for _ in range(5): caps[d].read()
ths = [threading.Thread(target=run, args=(d,), daemon=True) for d in caps]
t0 = time.perf_counter(); [t.start() for t in ths]; time.sleep(5); stop.set(); dt = time.perf_counter() - t0
for t in ths: t.join(1)
print(f"{len(caps)} cameras MJPG, simultaneously for {dt:.1f}s:")
for d in caps:
    print(f"  {d}  {sizes[d]:9s} usb {usb_path(d):10s}  {counts[d] / dt:5.1f} fps  read failures {fails[d]}")
    caps[d].release()
