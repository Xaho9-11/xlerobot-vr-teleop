"""Rough camera latency (screen-to-app) on the robot laptop, by the blink method.
A fullscreen window on the laptop's own screen flips black/white at random times; the camera looks at
the screen; we time from the flip call to the first captured frame whose brightness crosses the midpoint.
Result = display scan-out + camera exposure + USB transfer + decode (+ one display frame of uncertainty).
It does NOT include network streaming or the headset; that comes later.
Usage (via cam_latency.sh so DISPLAY is set): cam_latency.py /dev/videoN [fourcc w h fps trials [exposure]]"""
import csv, os, random, statistics as st, sys, threading, time
import cv2
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame

dev = sys.argv[1]
fourcc, w, h, fps = (sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else ("MJPG", 1280, 720, 30)
trials = int(sys.argv[6]) if len(sys.argv) > 6 else 20
exposure = int(sys.argv[7]) if len(sys.argv) > 7 else None  # UVC exposure_time_absolute, 100 us units; None = auto

cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h); cap.set(cv2.CAP_PROP_FPS, fps)
# no CAP_PROP_BUFFERSIZE=1: a single V4L2 buffer throttled this camera to ~19 fps; the grab thread drains buffers anyway
if exposure is not None:
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # V4L2: 1 = manual
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    print(f"manual exposure {exposure} (x100us) -> reported {cap.get(cv2.CAP_PROP_EXPOSURE)}")
frames, stop = [], threading.Event()

def grab():
    while not stop.is_set():
        ok, fr = cap.read()
        if ok:
            t = time.perf_counter()  # timestamp first, then a cheap 40x40 centre patch (keeps the grab loop at full fps)
            hh, ww = fr.shape[:2]
            frames.append((t, float(fr[hh // 2 - 20: hh // 2 + 20, ww // 2 - 20: ww // 2 + 20].mean())))

threading.Thread(target=grab, daemon=True).start()
pygame.init()
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.mouse.set_visible(False)

def show(white):
    screen.fill((255, 255, 255) if white else (0, 0, 0)); pygame.display.flip(); return time.perf_counter()

state = False; show(state); time.sleep(2.0)
lo = st.mean(b for _, b in frames[-10:]); show(True); time.sleep(1.0); hi = st.mean(b for _, b in frames[-10:]); show(False); time.sleep(1.0)
print(f"brightness black {lo:.0f}  white {hi:.0f}  (need a clear gap; aim the camera at the screen centre)")
if hi - lo < 30:
    print("contrast too low, aborting"); stop.set(); pygame.quit(); sys.exit(1)
mid = (lo + hi) / 2
lat = []
for k in range(trials):
    pygame.event.pump()
    time.sleep(random.uniform(0.6, 1.2))
    state = not state
    n0 = len(frames); t_flip = show(state)
    t_end = t_flip + 1.0
    while time.perf_counter() < t_end:
        hit = next(((t, b) for t, b in frames[n0:] if t > t_flip and ((b > mid) == state)), None)
        if hit:
            lat.append((hit[0] - t_flip) * 1000); break
        time.sleep(0.005)
stop.set(); pygame.quit(); cap.release()
fd = [b - a for (a, _), (b, _) in zip(frames, frames[1:])]
print(f"camera {fourcc} {w}x{h}@{fps}: measured {1 / st.mean(fd):.1f} fps")
if lat:
    lat.sort()
    print(f"screen->app latency over {len(lat)}/{trials} flips: median {st.median(lat):.0f} ms, "
          f"min {lat[0]:.0f}, p90 {lat[int(len(lat) * .9) - 1]:.0f}, max {lat[-1]:.0f} ms")
    out = os.path.expanduser(f"~/fmp_logs/cam_latency_{time.strftime('%Y%m%d_%H%M%S')}_{w}x{h}.csv")
    with open(out, "w", newline="") as f:
        csv.writer(f).writerows([["latency_ms"]] + [[f"{x:.1f}"] for x in lat])
    print("log:", out)
