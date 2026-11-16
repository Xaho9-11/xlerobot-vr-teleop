"""Open several cameras at once, the way the recorder will, and report real fps and broken frames per camera
(2026-11-11: four USB 2.0 cameras on two USB 2.0 buses; checks whether a layout has enough bandwidth).
Saves one JPEG per camera to /tmp/camtest_<n>.jpg so the cameras can be told apart.
Usage: cam_bus_test.py SECONDS dev@WxH[@fourcc] [dev@WxH ...]"""
import sys, threading, time
import cv2

secs = float(sys.argv[1])
specs = []
for a in sys.argv[2:]:
    dev, size, *rest = a.split("@")
    specs.append((dev, size, rest[0] if rest else "MJPG"))

res = {}


def run(i, dev, size, fourcc):
    w, h = map(int, size.split("x"))
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h); cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        res[i] = dict(dev=dev, error="cannot open"); return
    t_end = time.time() + 1.0
    while time.time() < t_end:  # warm-up (auto exposure)
        cap.read()
    n = bad = 0
    t0 = time.time()
    last = None
    while time.time() - t0 < secs:
        ok, f = cap.read()
        if not ok or f is None:
            bad += 1; continue
        n += 1; last = f
    if last is not None:
        cv2.imwrite(f"/tmp/camtest_{i}.jpg", cv2.resize(last, (320, int(320 * last.shape[0] / last.shape[1]))))
    res[i] = dict(dev=dev.split("/")[-1], size=f"{int(cap.get(3))}x{int(cap.get(4))}", fps=round(n / secs, 1), failed=bad,
                  mean=round(float(last.mean()), 1) if last is not None else None)
    cap.release()


ts = [threading.Thread(target=run, args=(i, *s)) for i, s in enumerate(specs)]
[t.start() for t in ts]; [t.join() for t in ts]
for i in sorted(res):
    print(i, res[i])
