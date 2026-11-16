"""Offline: SpringFollower feed-forward scale and gap extrapolation vs stop overshoot and lag.
Extrapolation: when no new sample for > 1 loop tick, the target is advanced by target_vel * gap,
capped at EXTRAP_MAX seconds. Usage: replay_ff.py [log.csv]"""
import csv, glob, math, sys
ARGS = sys.argv[1:]; sys.argv = sys.argv[:1]
import head_follow as hf
import replay_follower as rf

path = ARGS[0] if ARGS else sorted(glob.glob(__import__("os").path.expanduser("~/fmp_logs/head_follow_reachy_*.csv")))[-1]
rf.rows[:] = [r for r in csv.DictReader(open(path)) if r["state"] == "ok"]
DT = 0.02


def run(filt, tv, newflags, ff, extrap_max):
    s = hf.SpringFollower(w=25.0, ff=ff); out = []; since = 0.0
    for f, v, n in zip(filt, tv, newflags):
        since = 0.0 if n else since + DT
        tgt = f + v * min(since, extrap_max) if extrap_max else f
        out.append(s.step(tgt, v, DT))
    return out


newflags = [r["new_sample"] == "1" for r in rf.rows]
print(path)
for axis in ("yaw", "pitch"):
    raw, filt, tv = rf.inputs(axis)
    print(f"\n{axis}: {'variant':18s} {'lag':>6s} {'jerk p95':>9s}  stop overshoot mean/max (n)")
    lag, amax, jp, om, ox, no = rf.metrics(raw, filt, raw)
    print(f"   {'[input] raw head':18s} {lag:4.0f}ms {jp:9.0f}   {om:5.2f} / {ox:5.2f} deg ({no})")
    for ff in (1.0, 0.8, 0.6, 0.4, 0.0):
        for ex in (0.0, 0.06):
            cmd = run(filt, tv, newflags, ff, ex)
            lag, amax, jp, om, ox, no = rf.metrics(raw, filt, cmd)
            print(f"   ff {ff:.1f} extrap {ex * 1000:3.0f}ms {lag:4.0f}ms {jp:9.0f}   {om:5.2f} / {ox:5.2f} deg ({no})")
