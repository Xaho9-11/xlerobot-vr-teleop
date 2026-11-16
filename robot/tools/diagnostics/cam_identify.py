"""Identify which /dev/video node is which physical camera without looking at images:
records mean brightness of every external camera for DURATION s; cover one lens meanwhile and the
node whose brightness drops most is that camera. Usage: cam_identify.py [duration_s]"""
import glob, os, sys, threading, time
import cv2

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20
nodes = []
for d in sorted(glob.glob("/dev/video*")):
    name = open(f"/sys/class/video4linux/{os.path.basename(d)}/name").read()
    if "Integrated" in name:
        continue
    c = cv2.VideoCapture(d, cv2.CAP_V4L2)
    if c.isOpened():
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); c.set(3, 640); c.set(4, 480)
        port = os.path.basename(os.path.dirname(os.path.realpath(f"/sys/class/video4linux/{os.path.basename(d)}/device")))
        nodes.append((d, port, c))
series = {d: [] for d, _, _ in nodes}
stop = threading.Event()

def run(d, c):
    while not stop.is_set():
        ok, fr = c.read()
        if ok:
            series[d].append((time.perf_counter(), float(fr.mean())))

ths = [threading.Thread(target=run, args=(d, c), daemon=True) for d, _, c in nodes]
t0 = time.perf_counter(); [t.start() for t in ths]
print(f"recording {dur:.0f}s: cover ONE camera lens for ~5 s now", flush=True)
while time.perf_counter() - t0 < dur:
    time.sleep(1)
    print(f"  t={time.perf_counter() - t0:4.0f}s  " + "  ".join(
        f"{d}:{series[d][-1][1]:5.0f}" if series[d] else f"{d}:  ---" for d, _, _ in nodes), flush=True)
stop.set()
print("brightness drop (median of first 2 s minus minimum):")
for d, port, c in nodes:
    v = [b for _, b in series[d]]
    head = sorted(b for t, b in series[d] if t - t0 < 2) or v
    drop = head[len(head) // 2] - min(v) if v else 0
    print(f"  {d}  usb {port:9s} drop {drop:5.0f}")
    c.release()
