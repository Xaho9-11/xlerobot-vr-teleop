"""Episode recorder of the robot core (2026-11-10; requirements from study-protocol.md and part-a-session-protocol.md).

Raw, lossless research record - nothing is smoothed, trimmed or resampled. One folder per episode:
    ~/fmp_data/sessions/<YYYYMMDD>_<participant>/<NNN>_<scenario>_v<version>/
        meta.json      session/episode info: participant, scenario, style id, version, final/repeat flags, reason,
                       start/end wall time, arms, mapping parameters, discarded flag (episodes are never deleted)
        robot.csv      every control tick (~50 Hz): wall time, mode, per joint measured + commanded (pose degrees,
                       gripper %), torque, teleop engagement and head state
        vr.jsonl       every headset message as received: wall time, app send time, head + controller poses/buttons
        events.jsonl   automatic (first motion, gripper open, back at ready pose, target zone: arrive / reenter /
                       leave, zone.py) and manual (researcher) events; meta.json keeps the zone used (target_zone)
        cam_*.mp4/.csv written by video_daemon.py (frames + per-frame capture time), same wall clock
A LeRobot dataset is exported from these folders by a separate script, so the raw record never depends on it.
"""
import csv, json, os, re, time

from zone import ZoneTracker

DATA = os.path.expanduser("~/fmp_data/sessions")


def write_meta_file(d, meta):
    tmp = os.path.join(d, "meta.json.tmp")
    with open(tmp, "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    os.replace(tmp, os.path.join(d, "meta.json"))


def slug(s, n=24):
    s = re.sub(r"[^\w\-]+", "-", str(s or "").strip(), flags=re.UNICODE).strip("-")
    return (s or "x")[:n]


class Recorder:
    def __init__(self, joints):
        self.joints = joints
        self.session = dict(participant="P00", scenario="test", style="", operator="researcher", note="")
        self.ep = None              # open episode: dict(dir, meta, files...)
        self.last = None            # meta of the last closed episode (for after-the-fact flags)

    # ---------------------------------------------------------------- session
    def set_session(self, **kw):
        for k in ("participant", "scenario", "style", "operator", "note"):
            if k in kw and kw[k] is not None:
                self.session[k] = str(kw[k]).strip()

    def session_dir(self):
        return os.path.join(DATA, f"{time.strftime('%Y%m%d')}_{slug(self.session['participant'])}")

    def next_index_and_version(self, sdir):
        idx, ver = 0, 0
        if os.path.isdir(sdir):
            for d in os.listdir(sdir):
                m = re.match(r"(\d+)_(.+)_v(\d+)$", d)
                if m:
                    idx = max(idx, int(m.group(1)))
                    try:
                        mm = json.load(open(os.path.join(sdir, d, "meta.json")))
                        if mm.get("scenario") == self.session["scenario"] and not mm.get("discarded"):
                            ver = max(ver, int(mm.get("version", 0)))
                    except (OSError, ValueError):
                        pass
        return idx + 1, ver + 1

    # ---------------------------------------------------------------- episode
    def start(self, info):
        if self.ep:
            return None
        sdir = self.session_dir()
        idx, ver = self.next_index_and_version(sdir)
        d = os.path.join(sdir, f"{idx:03d}_{slug(self.session['scenario'])}_v{ver}")
        os.makedirs(d, exist_ok=True)
        meta = dict(self.session, episode=idx, version=ver, final=False, repeat_of_final=False, reason="",
                    discarded=False, t_start=time.time(), t_end=None, **info)
        rf = open(os.path.join(d, "robot.csv"), "w", newline="")
        rw = csv.writer(rf)
        rw.writerow(["t", "mode"] + [f"{j}.{k}" for j in self.joints for k in ("pos", "goal", "torque")]
                    + ["left_engaged", "right_engaged", "head_state"])
        self.ep = dict(dir=d, meta=meta, rf=rf, rw=rw, vf=open(os.path.join(d, "vr.jsonl"), "w"),
                       ef=open(os.path.join(d, "events.jsonl"), "w"), auto=dict(moved=False, gripper_min={}, at_ready=False),
                       zone=ZoneTracker(info["target_zone"]) if info.get("target_zone") else None)
        self.write_meta()
        self.event("episode_start", "开始录制", source="auto")
        return d

    def stop(self):
        if not self.ep:
            return None
        self.event("episode_end", "结束录制", source="auto")
        self.ep["meta"]["t_end"] = time.time()
        self.write_meta()
        for k in ("rf", "vf", "ef"):
            self.ep[k].close()
        self.last = dict(dir=self.ep["dir"], meta=self.ep["meta"])
        d, self.ep = self.ep["dir"], None
        return d

    def write_meta(self, target=None, **changes):
        """merge `changes` into meta.json and write it atomically. Outside a running episode the file on disk is
        read first, so edits made in the console's data view (server.py) are never overwritten by the core."""
        t = target or self.ep
        if not os.path.isdir(t["dir"]):  # moved to the trash from the console meanwhile
            return
        if t is not self.ep:
            try:
                t["meta"] = json.load(open(os.path.join(t["dir"], "meta.json")))
            except (OSError, ValueError):
                pass
        t["meta"].update(changes)
        write_meta_file(t["dir"], t["meta"])

    def annotate(self, **kw):
        """flags set after (or during) an episode: final, repeat_of_final, reason, discarded, style, note."""
        t = self.ep or self.last
        if not t:
            return False
        ch = {k: bool(kw[k]) for k in ("final", "repeat_of_final", "discarded") if k in kw}
        ch.update({k: str(kw[k]) for k in ("reason", "style", "note") if kw.get(k) is not None})
        self.write_meta(t, **ch)
        return True

    # ---------------------------------------------------------------- streams
    def event(self, kind, label="", source="manual", t=None, **extra):
        """t: the moment the event happened, when it is confirmed later (target zone: after the dwell)."""
        if not self.ep:
            return
        if kind not in ("episode_start", "episode_end"):  # shown on the phone remote (remote.html)
            self.ep.setdefault("evlog", []).append([round((t or time.time()) - self.ep["meta"]["t_start"], 1), label, source])
        self.ep["ef"].write(json.dumps(dict(t=t or time.time(), kind=kind, label=label, source=source, **extra),
                                       ensure_ascii=False) + "\n")
        self.ep["ef"].flush()

    def tick(self, mode, joints, engaged, head_state, ready_pose_deg=None, tcp=None):
        """joints: name -> dict(pos, goal, torque) in pose degrees / gripper %. tcp: tool point of the target-zone
        arm in its base frame (zone.py), None without a zone."""
        if not self.ep:
            return
        now = time.time()
        if self.ep["zone"] is not None:  # zone.py: hysteresis + dwell, first entry = 到达目标上方
            for e in self.ep["zone"].update(now, tcp):
                if e["kind"] == "target_arrive":
                    self.ep["arrived"] = round(e["t"] - self.ep["meta"]["t_start"], 1)
                self.event(e.pop("kind"), e.pop("label"), source="auto", **e)
        row = [f"{now:.4f}", mode]
        for j in self.joints:
            v = joints.get(j) or {}
            row += ["" if v.get("pos") is None else f"{v['pos']:.3f}", "" if v.get("goal") is None else f"{v['goal']:.3f}",
                    int(bool(v.get("torque")))]
        row += [int(bool(engaged.get("left"))), int(bool(engaged.get("right"))), head_state or ""]
        self.ep["rw"].writerow(row)
        self.auto_events(joints, engaged, ready_pose_deg)

    def vr(self, msg, recv_wall):
        if self.ep and msg is not None:
            self.ep["vf"].write(json.dumps(dict(t=recv_wall, msg=msg), separators=(",", ":")) + "\n")

    def auto_events(self, joints, engaged, ready):
        """part-a-session-protocol §8: start moving = first motion command (an arm engaged); release = gripper
        opening command (goal up >= 15 % from its lowest since the episode start); back = arms within 5 deg of the
        ready pose after having moved."""
        a = self.ep["auto"]
        if not a["moved"] and any(engaged.values()):
            a["moved"] = True; a["at_ready"] = False
            self.event("motion_start", "开始移动", source="auto")
        for side in ("left", "right"):
            g = (joints.get(f"{side}_arm_gripper") or {}).get("goal")
            if g is None or not engaged.get(side):
                continue
            lo = min(a["gripper_min"].get(side, g), g)
            a["gripper_min"][side] = lo
            if g - lo >= 15:
                self.event("gripper_open", "夹爪张开", source="auto", side=side, goal=round(g, 1))
                a["gripper_min"][side] = g  # re-arm after the next closing
        if a["moved"] and ready:
            arm = [j for j in ready if j in joints and joints[j].get("pos") is not None and joints[j].get("torque")]
            near = arm and all(abs(joints[j]["pos"] - ready[j]) < 5 for j in arm)
            if near and not a["at_ready"]:
                a["at_ready"] = True
                self.event("back_at_ready", "回到就绪姿态", source="auto")
            elif not near:
                a["at_ready"] = False

    def status(self):
        if self.last and not os.path.isdir(self.last["dir"]):  # trashed from the console
            self.last = None
        if not self.ep:
            return dict(on=False, session=self.session, last=self.last["meta"] if self.last else None)
        m = self.ep["meta"]
        return dict(on=True, session=self.session, episode=m["episode"], version=m["version"],
                    elapsed=round(time.time() - m["t_start"], 1), dir=os.path.basename(self.ep["dir"]),
                    zone=self.ep["zone"] is not None, arrived=self.ep.get("arrived"),
                    evlog=self.ep.get("evlog", [])[-6:])
