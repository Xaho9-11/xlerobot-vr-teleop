"""Export recorded episodes (recorder.py raw folders) to a LeRobot dataset for training (2026-11-10).

The raw record stays the source of truth; this makes a derived copy:
  * a uniform timeline at --fps over the span where every chosen camera has frames;
  * per tick the nearest-earlier camera frame of each camera and the nearest-earlier robot.csv row;
  * observation.state = measured joints, action = commanded joints, both in pose degrees (0 = calibration pose;
    gripper %), for the chosen arms plus the head (the head follows the headset and moves the head camera);
  * images: head resized to --head-size (default 640x360, keeps 16:9), wrists 640x480;
  * task string = "<scenario>" (+ " | style <id>" when a style is set) so a style-conditioned policy can use it.
Discarded episodes are skipped unless --include-discarded. Writes <out>/export_manifest.json listing the source
folders, so every training run can be traced back to raw episodes.

Usage: export_lerobot.py --out ~/fmp_datasets/handover_v1 --arms left SESSION_OR_EPISODE_DIR [...]
"""
import argparse, csv, glob, json, os, sys, time
import numpy as np

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
DATA = os.path.expanduser("~/fmp_data/sessions")


def episode_dirs(paths):
    out = []
    for p in paths:
        p = os.path.expanduser(p)
        if not os.path.isabs(p):
            p = os.path.join(DATA, p)
        if os.path.isfile(os.path.join(p, "meta.json")):
            out.append(p)
        else:
            out += sorted(os.path.dirname(m) for m in glob.glob(os.path.join(p, "*", "meta.json")))
    return out


def load_robot(path, joints):
    ts, pos, goal = [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                p = [float(r[f"{j}.pos"]) for j in joints]
                g = [float(r[f"{j}.goal"]) for j in joints]
            except (KeyError, ValueError):
                continue  # a tick with missing readings (bus hiccup)
            ts.append(float(r["t"])); pos.append(p); goal.append(g)
    return np.array(ts), np.array(pos, np.float32), np.array(goal, np.float32)


class CamReader:
    """sequential reader: frame(t) = the last frame captured at or before t (frames only move forward)."""

    def __init__(self, d, name, size):
        import av
        self.ts = [float(l.split(",")[1]) for l in open(os.path.join(d, f"cam_{name}.csv")).read().split("\n")[1:] if l]
        self.c = av.open(os.path.join(d, f"cam_{name}.mp4"))
        self.frames = self.c.decode(video=0)
        self.size, self.i, self.cur = size, -1, None

    def frame(self, t):
        import cv2
        while self.i + 1 < len(self.ts) and self.ts[self.i + 1] <= t:
            f = next(self.frames, None)
            if f is None:
                break
            self.i += 1
            self.cur = cv2.resize(f.to_ndarray(format="rgb24"), self.size, interpolation=cv2.INTER_AREA)
        return self.cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+", help="session or episode folders (absolute, or relative to ~/fmp_data/sessions)")
    ap.add_argument("--out", required=True, help="dataset root folder (must not exist)")
    ap.add_argument("--repo-id", default=None, help="name stored in the dataset (default: folder name)")
    ap.add_argument("--arms", choices=["left", "right", "both"], default="left")
    ap.add_argument("--cams", default="head,left,right")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--head-size", default="640x360")
    ap.add_argument("--include-discarded", action="store_true")
    a = ap.parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    sides = ["left", "right"] if a.arms == "both" else [a.arms]
    joints = [f"{s}_arm_{j}" for s in sides for j in ARM] + ["head_motor_1", "head_motor_2"]
    cams = [c for c in a.cams.split(",") if c]
    hw, hh = (int(v) for v in a.head_size.split("x"))
    size = {c: ((hw, hh) if c == "head" else (640, 480)) for c in cams}
    eps = episode_dirs(a.paths)
    chosen = []
    for d in eps:
        m = json.load(open(os.path.join(d, "meta.json")))
        if m.get("discarded") and not a.include_discarded:
            print("skip (discarded):", d); continue
        if not m.get("t_end"):
            print("skip (unfinished):", d); continue
        if any(not os.path.isfile(os.path.join(d, f"cam_{c}.mp4")) for c in cams):
            print("skip (camera missing):", d); continue
        chosen.append((d, m))
    if not chosen:
        sys.exit("nothing to export")

    features = {
        "observation.state": dict(dtype="float32", shape=(len(joints),), names=joints),
        "action": dict(dtype="float32", shape=(len(joints),), names=joints),
    }
    for c in cams:
        w, h = size[c]
        features[f"observation.images.{c}"] = dict(dtype="video", shape=(h, w, 3), names=["height", "width", "channels"])
    out = os.path.expanduser(a.out)
    ds = LeRobotDataset.create(repo_id=a.repo_id or os.path.basename(out.rstrip("/")), fps=a.fps, features=features,
                               root=out, robot_type="xlerobot", use_videos=True)
    manifest = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), fps=a.fps, arms=a.arms, cams=cams, joints=joints,
                    units="pose degrees (0 = calibration pose), gripper percent", episodes=[])
    for d, m in chosen:
        ts, pos, goal = load_robot(os.path.join(d, "robot.csv"), joints)
        readers = {c: CamReader(d, c, size[c]) for c in cams}
        t_from = max([ts[0]] + [r.ts[0] for r in readers.values()])
        t_to = min([ts[-1]] + [r.ts[-1] for r in readers.values()])
        task = m.get("scenario", "task") + (f" | style {m['style']}" if m.get("style") else "")
        n = 0
        for k in range(int((t_to - t_from) * a.fps) + 1):
            t = t_from + k / a.fps
            i = max(0, int(np.searchsorted(ts, t, side="right")) - 1)
            frame = {"observation.state": pos[i], "action": goal[i], "task": task}
            for c, r in readers.items():
                frame[f"observation.images.{c}"] = r.frame(t)
            ds.add_frame(frame); n += 1
        ds.save_episode()
        for r in readers.values():
            r.c.close()
        manifest["episodes"].append(dict(source=os.path.relpath(d, DATA), frames=n, task=task, style=m.get("style"),
                                         version=m.get("version"), final=m.get("final"),
                                         repeat_of_final=m.get("repeat_of_final")))
        print(f"exported {os.path.relpath(d, DATA)}: {n} frames, task '{task}'", flush=True)
    ds.finalize()
    json.dump(manifest, open(os.path.join(out, "export_manifest.json"), "w"), ensure_ascii=False, indent=1)
    print("dataset written:", out)


if __name__ == "__main__":
    main()
