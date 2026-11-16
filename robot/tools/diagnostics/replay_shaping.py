"""Offline replay: feed a recorded head_follow log's raw head angles through the OLD (deadband+EMA+
rate limit) and NEW (One Euro + speed/accel shaper) pipelines; compare smoothness and lag. No robot."""
import csv, glob, math, sys
sys.argv += [] ; import head_follow as hf

path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(__import__("os").path.expanduser("~/fmp_logs/head_follow_xlevr_*.csv")))[-1]
rows = [r for r in csv.DictReader(open(path)) if r["state"] == "ok"]
HZ, dt = 50.0, 0.02

def old(raw, new, hts):
    alpha = 1 - math.exp(-2 * math.pi * 6.0 * dt); f = 0.0; c = 0.0; out = []
    for x in raw:
        if abs(x - f) > 0.3: f += alpha * (x - f)
        s = 120 * dt; c += max(-s, min(s, f - c)); out.append(c)
    return out

def newp(raw, new, hts, min_cutoff=1.0, beta=0.02, k=20.0, vmax=180.0, amax=1200.0):
    flt = hf.OneEuro(min_cutoff, beta); f = raw[0]; c = v = 0.0; last = None; out = []
    for x, n, h in zip(raw, new, hts):
        if n:
            hs = h / 1000 if h else None
            sdt = hs - last if (hs and last) else dt
            sdt = sdt if 0.002 < sdt < 0.5 else dt
            last = hs; f = flt(x, sdt)
        vd = max(-vmax, min(vmax, k * (f - c))); dv = amax * dt
        v += max(-dv, min(dv, vd - v)); c += v * dt; out.append(c)
    return out

def stats(name, raw, cmd):
    vel = [(b - a) / dt for a, b in zip(cmd, cmd[1:])]
    acc = [(b - a) / dt for a, b in zip(vel, vel[1:])]
    n = len(raw) - 30
    lag = min(range(30), key=lambda k: sum((cmd[i + k] - raw[i]) ** 2 for i in range(n)))
    rms = math.sqrt(sum((cmd[i + lag] - raw[i]) ** 2 for i in range(n)) / n)
    acc_s = sorted(map(abs, acc))
    rev = sum(1 for a, b in zip(vel, vel[1:]) if a * b < 0 and abs(a - b) > 100)
    print(f"  {name:4s} lag {lag * 20:4.0f} ms | rms vs raw (lag-aligned) {rms:5.2f} deg | |accel| p95 {acc_s[int(len(acc_s) * .95)]:6.0f} "
          f"max {acc_s[-1]:7.0f} deg/s2 | snap reversals (|dv|>100 deg/s in one tick) {rev}")

new = [r["new_sample"] == "1" for r in rows]
hts = [float(r["headset_ts_ms"]) if r["headset_ts_ms"] else None for r in rows]
print(path)
for ax in ("yaw", "pitch"):
    raw = [float(r[f"raw_{ax}"]) for r in rows]
    print(ax)
    stats("old", raw, old(raw, new, hts))
    stats("new", raw, newp(raw, new, hts))
