"""Offline study of 'the robot head stops twice': input smoothing at low speed and a stillness lock,
on a recorded head_follow log. Pipeline = One Euro (min_cutoff, beta) -> optional stillness lock ->
SpringFollower (w=25). Metrics on the follower output (what the servo is asked to do):
  lag     best-fit shift vs raw while moving
  stopgo  share of head stops where the command slows below 3 deg/s and then speeds up above 8 deg/s
  settle  median time after the head stops until the command stays within 0.3 deg of its final value
  still   RMS command speed during still periods (head speed < 3 deg/s on the smoothed input)
Usage: replay_stillness.py [log.csv]"""
import csv, glob, math, statistics as st, sys
ARGS = sys.argv[1:]; sys.argv = sys.argv[:1]
import head_follow as hf

path = ARGS[0] if ARGS else sorted(glob.glob(__import__("os").path.expanduser("~/fmp_logs/head_follow_reachy_*.csv")))[-1]
rows = [r for r in csv.DictReader(open(path)) if r["state"] == "ok"]
DT = 0.02


def simulate(axis, min_cutoff, beta, lock=None):
    """lock = (enter_speed, enter_time_s, release_deg) or None."""
    flt, last, f, tv, prev_f = hf.OneEuro(min_cutoff, beta), None, None, 0.0, None
    spring = hf.SpringFollower(w=25.0)
    held, slow_t, raw_s, cmd_s, sm_in = None, 0.0, [], [], []
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
        f_use = f if f is not None else raw
        target, tv_use = f_use, tv
        if lock:
            enter_v, enter_t, release = lock
            if held is None:
                slow_t = slow_t + DT if abs(tv) < enter_v else 0.0
                if slow_t >= enter_t:
                    held = f_use
            elif abs(f_use - held) > release or abs(tv) > 3 * enter_v:
                held, slow_t = None, 0.0
            if held is not None:
                target, tv_use = held, 0.0
        raw_s.append(raw); cmd_s.append(spring.step(target, tv_use, DT)); sm_in.append(f_use)
    return raw_s, cmd_s, sm_in


def metrics(raw, cmd, sm_in):
    n = len(raw) - 30
    lag = min(range(30), key=lambda k: sum((cmd[i + k] - raw[i]) ** 2 for i in range(n)))
    sm = [sum(sm_in[max(0, i - 9):i + 1]) / len(sm_in[max(0, i - 9):i + 1]) for i in range(len(sm_in))]
    sv = [(b - a) / DT for a, b in zip(sm, sm[1:])]
    cv = [(b - a) / DT for a, b in zip(cmd, cmd[1:])]
    ev, moving = [], False
    for i in range(len(sv) - 40):
        if abs(sv[i]) > 40: moving = True
        elif moving and abs(sv[i]) < 5: moving = False; ev.append(i)
    sg, settle = 0, []
    for i in ev:
        stopped, hit = False, False
        for s in cv[i - 5:i + 30]:
            if abs(s) < 3: stopped = True
            elif stopped and abs(s) > 8: hit = True; break
        sg += hit
        fin = cmd[i + 35]
        k = next((j for j in range(i, i + 35) if all(abs(c - fin) < 0.3 for c in cmd[j:i + 35])), i + 35)
        settle.append((k - i) * DT * 1000)
    still = [cv[i] for i in range(len(cv)) if abs(sv[min(i, len(sv) - 1)]) < 3]
    rms_still = math.sqrt(sum(x * x for x in still) / len(still)) if still else float("nan")
    return lag * DT * 1000, sg / max(1, len(ev)) * 100, st.median(settle) if settle else 0, rms_still, len(ev)


CANDS = [("current  mc2.0 b0.05", 2.0, 0.05, None),
         ("mc1.0 b0.05", 1.0, 0.05, None),
         ("mc0.5 b0.05", 0.5, 0.05, None),
         ("mc0.5 b0.10", 0.5, 0.10, None),
         ("mc1.0 b0.05 +lock", 1.0, 0.05, (4.0, 0.12, 1.5)),
         ("mc0.5 b0.10 +lock", 0.5, 0.10, (4.0, 0.12, 1.5)),
         ("mc1.0 b0.10 +lock2", 1.0, 0.10, (5.0, 0.10, 2.0))]

if __name__ == "__main__":
    print(path)
    for axis in ("yaw", "pitch"):
        print(f"\n{axis}: {'variant':22s} {'lag':>6s} {'stop-go':>8s} {'settle':>8s} {'still rms':>10s}")
        for name, mc, b, lock in CANDS:
            lag, sg, settle, rs, n = metrics(*simulate(axis, mc, b, lock))
            print(f"   {name:22s} {lag:4.0f}ms {sg:7.0f}% {settle:6.0f}ms {rs:8.2f}d/s  (stops {n})")
