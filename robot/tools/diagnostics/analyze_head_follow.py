"""Summarise a head_follow CSV: loop rate, sample ages, and lag/error of each pipeline stage.
Lags are best-fit shifts at loop resolution (1/hz), measured on the robot clock only."""
import csv, glob, math, statistics as st, sys

path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(__import__("os").path.expanduser("~/fmp_logs/head_follow_*.csv")))[-1]
rows = [r for r in csv.DictReader(open(path)) if r["state"] == "ok"]
t = [float(r["t"]) for r in rows]
dt = [b - a for a, b in zip(t, t[1:])]
ages = [float(r["sample_age_ms"]) for r in rows if r["sample_age_ms"]]
print(f"{path}\nok ticks {len(rows)}, loop {1 / st.mean(dt):.1f} Hz (dt mean {st.mean(dt) * 1000:.1f} ms, "
      f"p95 {sorted(dt)[int(len(dt) * .95)] * 1000:.1f} ms, max {max(dt) * 1000:.1f} ms)")
print(f"sample age at use: mean {st.mean(ages):.1f} ms, p95 {sorted(ages)[int(len(ages) * .95)]:.1f} ms")

def lag(a, b, max_k=25):
    """samples by which b trails a (best least-squares shift)."""
    n = len(a) - max_k
    return min(range(max_k), key=lambda k: sum((b[i + k] - a[i]) ** 2 for i in range(n)))

per = st.mean(dt) * 1000
for axis in ("yaw", "pitch"):
    raw = [float(r[f"raw_{axis}"]) for r in rows]
    cmd = [float(r[f"cmd_{axis}"]) for r in rows]
    act = [float(r[f"present_{axis}"]) for r in rows]
    k1, k2, k3 = lag(raw, cmd), lag(cmd, act), lag(raw, act)
    rms = math.sqrt(st.mean([(a - c) ** 2 for a, c in zip(act, cmd)]))
    print(f"{axis:5s} range raw {min(raw):+6.1f}..{max(raw):+6.1f} | lag raw->cmd {k1 * per:4.0f} ms (filter/rate limit), "
          f"cmd->servo {k2 * per:4.0f} ms, total raw->servo {k3 * per:4.0f} ms | rms(servo-cmd) {rms:.2f} deg")
