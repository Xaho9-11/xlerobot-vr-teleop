"""Offline comparison of motion shapers on a recorded head_follow log (no robot needed).

All candidates get the same input: One Euro filtered head angle (updated on new samples, timed by the
headset send time) plus an 8 Hz low-passed velocity estimate. Candidates:
  old     2026-10-14 shaper: v_des = 30*e, |v|<=180, |a|<=1200 (user: jerky at start/stop, 2026-10-20)
  smooth  SmoothFollower: jerk-limited, braking-aware speed, velocity feed-forward
  spring  SpringFollower: critically damped spring-damper (w rad/s) with accel/jerk limits
Metrics: lag vs raw (best shift), peak |accel|, jerk p95, and overshoot at head stops. Stops are found on
a 0.2 s moving average of the filtered target (raw/filtered are step-like at the ~30 Hz sample rate):
its speed falls below 5 deg/s after exceeding 40 deg/s; overshoot = travel beyond where it then settles.
Usage: replay_follower.py [log.csv]
"""
import csv, glob, math, sys

ARGS = sys.argv[1:]
sys.argv = sys.argv[:1]
import head_follow as hf

path = ARGS[0] if ARGS else sorted(glob.glob(__import__("os").path.expanduser("~/fmp_logs/head_follow_reachy_*.csv")))[-1]
rows = [r for r in csv.DictReader(open(path)) if r["state"] == "ok"]
DT = 0.02


def inputs(axis):
    flt, last, f, tv, prev_f = hf.OneEuro(2.0, 0.05), None, None, 0.0, None
    raw_s, filt_s, tv_s = [], [], []
    for r in rows:
        raw = float(r[f"raw_{axis}"])
        if r["new_sample"] == "1":
            h = float(r["headset_ts_ms"]) / 1000 if r["headset_ts_ms"] else None
            sdt = (h - last) if (h and last) else DT
            sdt = sdt if 0.002 < sdt < 0.5 else DT
            last = h
            f = flt(raw, sdt)
            if prev_f is not None:
                a = 1 - math.exp(-2 * math.pi * 8.0 * sdt)
                tv += a * ((f - prev_f) / sdt - tv)
            prev_f = f
        raw_s.append(raw); filt_s.append(f if f is not None else raw); tv_s.append(tv)
    return raw_s, filt_s, tv_s


def run_old(filt, tv, k=30.0, vmax=180.0, amax=1200.0):
    c = v = 0.0; out = []
    for f in filt:
        vd = max(-vmax, min(vmax, k * (f - c))); dv = amax * DT
        v += max(-dv, min(dv, vd - v)); c += v * DT; out.append(c)
    return out


def run(cls, filt, tv, **kw):
    s = cls(**kw)
    return [s.step(f, v, DT) for f, v in zip(filt, tv)]


def moving_avg(x, n=10):
    out, acc = [], 0.0
    for i, v in enumerate(x):
        acc += v - (x[i - n] if i >= n else 0.0)
        out.append(acc / min(i + 1, n))
    return out


def metrics(raw, filt, cmd):
    n = len(raw) - 30
    lag = min(range(30), key=lambda k: sum((cmd[i + k] - raw[i]) ** 2 for i in range(n)))
    vel = [(b - a) / DT for a, b in zip(cmd, cmd[1:])]
    acc = [(b - a) / DT for a, b in zip(vel, vel[1:])]
    jerk = sorted(abs((b - a) / DT) for a, b in zip(acc, acc[1:]))
    sm = moving_avg(filt)
    sv = [(b - a) / DT for a, b in zip(sm, sm[1:])]
    over, moving, sgn = [], False, 1.0
    for i in range(len(sv) - 40):
        if abs(sv[i]) > 40:
            moving, sgn = True, math.copysign(1, sv[i])
        elif moving and abs(sv[i]) < 5:
            moving = False
            final = sm[i + 20]
            over.append(max(0.0, max(sgn * (c - final) for c in cmd[i:i + 40])))
    return (lag * DT * 1000, max(map(abs, acc)), jerk[int(len(jerk) * .95)],
            sum(over) / len(over) if over else 0.0, max(over) if over else 0.0, len(over))


CANDS = [("old (2026-10-14)", None, {}),
         ("smooth J20k ff1.0", hf.SmoothFollower, dict(jerk=20000, ff=1.0)),
         ("spring w20", hf.SpringFollower, dict(w=20.0)),
         ("spring w25", hf.SpringFollower, dict(w=25.0)),
         ("spring w30", hf.SpringFollower, dict(w=30.0)),
         ("spring w35 a2500", hf.SpringFollower, dict(w=35.0, a_max=2500.0)),
         ("spring w30 J30k", hf.SpringFollower, dict(w=30.0, jerk=30000.0))]

if __name__ == "__main__":
    print(path)
    for axis in ("yaw", "pitch"):
        raw, filt, tv = inputs(axis)
        print(f"\n{axis}: {'shaper':20s} {'lag':>6s} {'max|acc|':>9s} {'jerk p95':>9s}  stop overshoot mean/max (n)")
        for name, ref in (("[input] raw head", raw), ("[input] filtered", filt)):
            lag, amax, jp, om, ox, no = metrics(raw, filt, ref)
            print(f"   {name:20s} {lag:4.0f}ms {amax:8.0f} {jp:9.0f}   {om:5.2f} / {ox:5.2f} deg ({no})")
        for name, cls, kw in CANDS:
            cmd = run_old(filt, tv) if cls is None else run(cls, filt, tv, **kw)
            lag, amax, jp, om, ox, no = metrics(raw, filt, cmd)
            print(f"   {name:20s} {lag:4.0f}ms {amax:8.0f} {jp:9.0f}   {om:5.2f} / {ox:5.2f} deg ({no})")
