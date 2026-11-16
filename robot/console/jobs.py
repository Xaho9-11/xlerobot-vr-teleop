"""Background jobs started from the console (2026-11-10): dataset export and ACT training.

Each job is a detached shell process (its own session, so restarting the console does not stop it), niced so the
robot core's control loop keeps the CPU. State lives in ~/fmp_logs/jobs/<id>/: cmd, log, pid, exit. The console
lists jobs by reading these folders, so nothing is lost across restarts.
"""
import json, os, re, shutil, signal, subprocess, time

JOBS = os.path.expanduser("~/fmp_logs/jobs")
DATASETS = os.path.expanduser("~/fmp_datasets")
SCRIPTS = os.path.expanduser("~/fmp_scripts")
PY = os.path.expanduser("~/miniforge3/envs/lerobot/bin/python")
CAM_SHAPES = {"head": [3, 360, 640], "left": [3, 480, 640], "right": [3, 480, 640]}  # export_lerobot.py sizes
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_\-]{0,47}$")


def _start(kind, title, shell_cmd, info):
    os.makedirs(JOBS, exist_ok=True)
    jid = time.strftime("%Y%m%d_%H%M%S") + "_" + kind
    d = os.path.join(JOBS, jid)
    os.makedirs(d)
    json.dump(dict(id=jid, kind=kind, title=title, cmd=shell_cmd, start=time.time(), **info),
              open(os.path.join(d, "job.json"), "w"), ensure_ascii=False, indent=1)
    wrapped = f"({shell_cmd}) > {d}/log 2>&1; echo $? > {d}/exit"
    p = subprocess.Popen(["nice", "-n", "10", "bash", "-c", wrapped], start_new_session=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    open(os.path.join(d, "pid"), "w").write(str(p.pid))
    return jid


def export(name, episodes, arms, cams, include_discarded=False):
    if not NAME_RE.match(name or ""):
        raise ValueError("数据集名称只能用字母、数字、- 和 _")
    out = os.path.join(DATASETS, name)
    if os.path.exists(out):
        raise ValueError("已有同名数据集")
    if not episodes:
        raise ValueError("没有选择录制")
    cams = [c for c in cams if c in CAM_SHAPES]
    args = " ".join(f"'{e}'" for e in episodes if "'" not in e)
    cmd = (f"{PY} {SCRIPTS}/console/export_lerobot.py {args} --out '{out}' --arms {arms} --cams {','.join(cams)}"
           + (" --include-discarded" if include_discarded else ""))
    return _start("export", f"导出 {name}（{len(episodes)} 条）", cmd,
                  dict(dataset=name, episodes=episodes, arms=arms, cams=cams))


def datasets():
    out = []
    if not os.path.isdir(DATASETS):
        return out
    for name in sorted(os.listdir(DATASETS)):
        d = os.path.join(DATASETS, name)
        info = os.path.join(d, "meta", "info.json")
        if not os.path.isfile(info):
            continue
        i = json.load(open(info))
        man = {}
        try:
            man = json.load(open(os.path.join(d, "export_manifest.json")))
        except (OSError, ValueError):
            pass
        feats = i.get("features", {})
        out.append(dict(name=name, episodes=i.get("total_episodes"), frames=i.get("total_frames"), fps=i.get("fps"),
                        cams=[k.split(".")[-1] for k, v in feats.items() if v.get("dtype") in ("video", "image")],
                        state_dim=(feats.get("observation.state", {}).get("shape") or [None])[0],
                        arms=man.get("arms"), created=man.get("created"),
                        sources=[e.get("source") for e in man.get("episodes", [])],
                        tasks=sorted({e.get("task") for e in man.get("episodes", []) if e.get("task")})))
    return out


def train(dataset, name, steps, batch, cams):
    ds = next((x for x in datasets() if x["name"] == dataset), None)
    if not ds:
        raise ValueError("找不到数据集")
    if not NAME_RE.match(name or ""):
        raise ValueError("训练名称只能用字母、数字、- 和 _")
    if os.path.exists(os.path.expanduser(f"~/fmp_train/runs/{name}")):
        raise ValueError("已有同名训练")
    steps, batch = int(steps), int(batch)
    if not (100 <= steps <= 300000 and 1 <= batch <= 32):
        raise ValueError("步数或批大小不合理")
    if any(t["kind"] == "train" and t["state"] == "running" for t in list_jobs()):
        raise ValueError("已有训练在进行（显卡只够跑一个）")
    cams = [c for c in cams if c in ds["cams"]] or ds["cams"]
    extra = ""
    if set(cams) != set(ds["cams"]):  # use only some cameras: give the policy its inputs explicitly
        feat = {"observation.state": {"type": "STATE", "shape": [ds["state_dim"]]}}
        feat.update({f"observation.images.{c}": {"type": "VISUAL", "shape": CAM_SHAPES[c]} for c in cams})
        extra = " '--policy.input_features=" + json.dumps(feat, separators=(",", ":")) + "'"
    label = f"{dataset} · {'+'.join(cams)} · 批大小{batch}"
    cmd = (f"cd {SCRIPTS}/act_test && REPO='local/{dataset}' ROOT='{DATASETS}/{dataset}' LABEL='{label}' "
           f"SAVE_FREQ=20000 bash run_train.sh '{name}' {steps} {batch}{extra} && "
           f"{PY} offline_eval.py '{name}' > ~/fmp_train/runs/'{name}'/eval.log 2>&1")
    return _start("train", f"训练 {name}（{dataset}，{steps} 步）", cmd,
                  dict(dataset=dataset, run=name, steps=steps, batch=batch, cams=cams))


def _alive(pid):
    try:
        os.kill(pid, 0); return True
    except OSError:
        return False


def list_jobs(limit=12):
    out = []
    if not os.path.isdir(JOBS):
        return out
    for jid in sorted(os.listdir(JOBS), reverse=True)[:limit]:
        d = os.path.join(JOBS, jid)
        try:
            j = json.load(open(os.path.join(d, "job.json")))
            pid = int(open(os.path.join(d, "pid")).read())
        except (OSError, ValueError):
            continue
        ex = open(os.path.join(d, "exit")).read().strip() if os.path.exists(os.path.join(d, "exit")) else None
        state = "running" if ex is None and _alive(pid) else ("done" if ex == "0" else
                 ("cancelled" if os.path.exists(os.path.join(d, "cancelled")) else "failed"))
        j.update(state=state, exit=ex, progress=_progress(j, d), tail=_tail(d))
        if state == "failed" and j["kind"] == "train":  # say why in plain words (2026-11-10: 3 cameras x batch 8 > 6 GB)
            try:
                if "OutOfMemoryError" in open(os.path.expanduser(f"~/fmp_train/runs/{j['run']}/train.log"), errors="replace").read():
                    j["tail"] = ["显存不足（6 GB）：减少相机路数，或把批大小改为 4"]
            except OSError:
                pass
        out.append(j)
    return out


def _tail(d, n=1):
    try:
        lines = [l for l in open(os.path.join(d, "log"), errors="replace").read().replace("\r", "\n").split("\n")
                 if l.strip() and not l.startswith(("Svt", "SVT", "-----"))]
        return lines[-n:]
    except OSError:
        return []


def _progress(j, d):
    if j["kind"] == "export":
        try:
            done = open(os.path.join(d, "log"), errors="replace").read().count("exported ")
        except OSError:
            done = 0
        return dict(done=done, total=len(j.get("episodes", [])))
    if j["kind"] == "train":
        log = os.path.expanduser(f"~/fmp_train/runs/{j['run']}/train.log")
        try:
            steps = re.findall(r"step:(\S+) smpl", open(log, errors="replace").read())
            return dict(done=len(steps) * 25, total=j["steps"])  # run_train.sh logs every 25 steps
        except OSError:
            return dict(done=0, total=j["steps"])
    return None


def cancel(jid):
    d = os.path.join(JOBS, os.path.basename(jid))
    pid = int(open(os.path.join(d, "pid")).read())
    open(os.path.join(d, "cancelled"), "w").close()
    try:
        os.killpg(pid, signal.SIGTERM)  # the whole session: shell, exporter / trainer, GPU sampler
    except OSError:
        pass
    j = json.load(open(os.path.join(d, "job.json")))
    if j["kind"] == "train":  # killpg stops run_train.sh before it can write exit: mark the run for the training page
        run = os.path.expanduser(f"~/fmp_train/runs/{j['run']}")
        if os.path.isdir(run):
            open(os.path.join(run, "cancelled"), "w").close()
    if j["kind"] == "export":  # a half-written dataset is useless and blocks the name
        shutil.rmtree(os.path.join(DATASETS, j["dataset"]), ignore_errors=True)
    return True
