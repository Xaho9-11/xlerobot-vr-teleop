"""Robot console web service (console-architecture.md, 2026-11-10). Serves the page, the twin model files and
a state stream (WebSocket /ws, ~30 Hz) relayed from the robot core (core.py, ws://127.0.0.1:8770); commands go back
the same way. Adds laptop status and whether the video program runs. --mock: simulated joints (UI work only).
Usage: server.py [--port 8080] [--mock]"""
import argparse, asyncio, glob, json, math, os, re, shutil, socket, subprocess, time
import jobs  # background export / training (jobs.py)
from aiohttp import web, WSMsgType

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
URDF_DIR = os.path.expanduser("~/code/XLeRobot/simulation/xlerobot")
MJCF_DIR = os.path.expanduser("~/code/XLeRobot/simulation/mujoco")
DATA = os.path.expanduser("~/fmp_data/sessions")  # recorder.py episodes
TRASH = os.path.expanduser("~/fmp_data/trash")    # episodes removed in the console (restorable, never auto-deleted)
EDITABLE = {"participant": str, "scenario": str, "style": str, "version": int, "final": bool, "repeat_of_final": bool,
            "discarded": bool, "reason": str, "note": str}


def episode_path(root, rel):
    """resolve an episode id ('<session>/<episode>') inside root, refusing anything outside it."""
    d = os.path.realpath(os.path.join(root, rel or ""))
    if not d.startswith(os.path.realpath(root) + os.sep) or not os.path.isfile(os.path.join(d, "meta.json")):
        raise web.HTTPNotFound(text="no such episode")
    return d  # three-omniwheel build (the URDF is the 2-wheel cart)
LIMITS = os.path.join(os.path.dirname(HERE), "joint_limits.json")

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
JOINTS = ["head_motor_1", "head_motor_2"] + [f"{s}_arm_{j}" for s in ("left", "right") for j in ARM]
MODES = ["idle", "hold", "ready", "teleop", "fold"]


def joint_limits():
    try:
        t = json.load(open(LIMITS))
    except (OSError, ValueError):
        return {}
    out = {m: v for m, v in t.get("pose_deg_limits_head", {}).items()}
    for s in ("left", "right"):
        for j, v in t.get("pose_deg_limits_both_arms", {}).items():
            out[f"{s}_arm_{j}"] = v
        out[f"{s}_arm_gripper"] = [0.0, 100.0]
    return out


class Laptop:
    """real machine status, refreshed every few seconds."""
    def __init__(self):
        self.data, self.t = {}, 0.0

    def get(self):
        if time.time() - self.t > 3:
            self.t = time.time()
            d = {}
            try:
                d["ac"] = open("/sys/class/power_supply/ADP0/online").read().strip() == "1"
                d["battery"] = int(open("/sys/class/power_supply/BAT0/capacity").read())
            except OSError:
                pass
            try:
                temps = [int(open(f).read()) / 1000 for f in
                         [os.path.join(z, "temp") for z in os.popen("ls -d /sys/class/thermal/thermal_zone*").read().split()]]
                d["cpu_temp"] = round(max(temps))
            except (OSError, ValueError):
                pass
            try:
                g = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu,utilization.gpu,memory.used",
                                    "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=2).stdout
                t, u, m = (int(x) for x in g.strip().split(","))
                d.update(gpu_temp=t, gpu_util=u, gpu_mem=m)
            except Exception:
                pass
            d["disk_free_gb"] = round(shutil.disk_usage("/").free / 1e9)
            self.data = d
        return self.data


class Mock:
    """simulated robot for the UI prototype: a mode cycle and smooth joint motion within the limits."""
    def __init__(self, limits):
        self.limits, self.t0 = limits, time.time()
        self.mode, self.events = "teleop", [(time.time(), "模拟数据：尚未连接机器人核心")]
        self.arms = "left"
        self.pos = {j: 0.0 for j in JOINTS}

    def command(self, cmd, arg=None):
        nxt = {"torque_on": "hold", "unfold": "ready", "start": "teleop", "stop": "fold", "release": "idle",
               "estop": "idle"}
        if cmd == "arms" and arg in ("left", "right", "both"):
            self.arms = arg
        elif cmd in nxt:
            self.mode = nxt[cmd]
        self.events.append((time.time(), {"torque_on": "上扭矩，保持当前位置", "unfold": "展开到就绪姿态",
                                          "start": "开始遥操作", "stop": "停止并收回折叠位", "release": "释放扭矩",
                                          "estop": "紧急释放：全部扭矩已关闭", "arms": f"手臂：{arg}"}.get(cmd, cmd)))
        self.events = self.events[-50:]

    def state(self):
        t = time.time() - self.t0
        goal = {}
        for k, j in enumerate(JOINTS):
            lo, hi = self.limits.get(j, [-60, 60])
            mid, amp = (lo + hi) / 2, (hi - lo) * 0.18
            if "gripper" in j:
                mid, amp = 40, 30
            if j.startswith("head"):
                mid, amp = 0, 25 if j.endswith("1") else 12
            moving = self.mode == "teleop" and (j.startswith("head") or self.arms in ("both", j.split("_")[0]))
            goal[j] = mid + amp * math.sin(0.5 * t + k * 0.9) * math.sin(0.17 * t + k) if moving else \
                (0.0 if self.mode in ("ready", "teleop", "hold") else self.pos[j])
            self.pos[j] += (goal[j] - self.pos[j]) * 0.12  # lag behind the goal, like the real followers
        joints = {j: dict(pos=round(self.pos[j], 2), goal=round(goal[j], 2), torque=self.mode != "idle",
                          temp=31 + (k % 5) * 2, volt=12.1, err=0) for k, j in enumerate(JOINTS)}
        return dict(t=time.time(), mock=True, mode=self.mode, arms=self.arms, joints=joints,
                    links=dict(bus1=True, bus2=True, cam_head=True, cam_left=True, cam_right=True,
                               quest=self.mode == "teleop", quest_age_ms=14, video=True),
                    events=[[round(a, 1), b] for a, b in self.events[-8:]])


class CoreLink:
    """Client of the robot core (ws://127.0.0.1:8770): latest state in, commands out. When the core is not running
    the console says so (no simulated data)."""
    def __init__(self, url="ws://127.0.0.1:8770"):
        self.url, self.ws, self.hello, self.last, self.since = url, None, None, None, time.time()

    async def run(self):
        import websockets
        while True:
            try:
                async with websockets.connect(self.url, max_size=None) as ws:
                    self.ws = ws
                    async for raw in ws:
                        m = json.loads(raw)
                        if m.get("type") == "hello":
                            self.hello = m
                        else:
                            self.last = m
            except (OSError, Exception):
                pass
            self.ws, self.last = None, None
            await asyncio.sleep(1.0)

    async def command(self, cmd, arg=None):
        if self.ws:
            await self.ws.send(json.dumps(dict(type="cmd", cmd=cmd, arg=arg)))

    def state(self):
        if self.last and time.time() - self.last.get("t", 0) < 2.0:
            return dict(self.last)
        return dict(type="state", core=False, mode="offline", motion=None, arms=None, joints={},
                    links=dict(bus1=False, bus2=False, quest=False),
                    events=[[round(time.time(), 1), "机器人核心未运行"]])


def video_up():
    """the video program listens on 8443 when it runs."""
    s = socket.socket(); s.settimeout(0.2)
    try:
        return s.connect_ex(("127.0.0.1", 8443)) == 0
    finally:
        s.close()


async def main_async(a):
    limits = joint_limits()
    src = Mock(limits) if a.mock else CoreLink()
    laptop = Laptop()
    clients = set()
    if not a.mock:
        asyncio.create_task(src.run())
    video = {"up": False, "t": 0.0}

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        clients.add(ws)
        hello = (src.hello if not a.mock else None) or dict(type="hello", joints=JOINTS, limits=limits)
        await ws.send_json(hello)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    m = json.loads(msg.data)
                    if m.get("type") == "cmd":
                        if a.mock:
                            src.command(m.get("cmd"), m.get("arg"))
                        else:
                            await src.command(m.get("cmd"), m.get("arg"))
        finally:
            clients.discard(ws)
        return ws

    async def pump():
        while True:
            if clients:
                if time.time() - video["t"] > 2:
                    video.update(up=video_up(), t=time.time())
                st = src.state(); st["laptop"] = laptop.get(); st["type"] = "state"
                st.setdefault("links", {})["video"] = video["up"]
                msg = json.dumps(st)
                for ws in list(clients):
                    try:
                        await ws.send_str(msg)
                    except ConnectionError:
                        clients.discard(ws)
            await asyncio.sleep(1 / 30)

    async def episodes(_):
        """recorded episodes (recorder.py layout), newest first, for the console's data view."""
        out = []
        for meta_path in glob.glob(os.path.join(DATA, "*", "*", "meta.json")):
            d = os.path.dirname(meta_path)
            try:
                m = json.load(open(meta_path))
            except (OSError, ValueError):
                continue
            files = {f: os.path.getsize(os.path.join(d, f)) for f in os.listdir(d)}
            out.append(dict(id=os.path.relpath(d, DATA), session=os.path.basename(os.path.dirname(d)),
                            participant=m.get("participant"), scenario=m.get("scenario"), style=m.get("style"),
                            episode=m.get("episode"), version=m.get("version"), final=m.get("final"),
                            repeat_of_final=m.get("repeat_of_final"), discarded=m.get("discarded"),
                            reason=m.get("reason"), note=m.get("note", ""), t_start=m.get("t_start"),
                            edits=len(m.get("edit_log", [])),
                            duration=(m["t_end"] - m["t_start"]) if m.get("t_end") else None,
                            cams=sorted(f[4:-4] for f in files if f.startswith("cam_") and f.endswith(".mp4")),
                            mb=round(sum(files.values()) / 1e6, 1), video=m.get("video")))
        out.sort(key=lambda e: e["t_start"] or 0, reverse=True)
        return web.json_response(dict(episodes=out, free_gb=round(shutil.disk_usage(DATA if os.path.isdir(DATA) else "/").free / 1e9)))

    async def episode_update(request):
        """edit an episode's labels; every change is appended to meta['edit_log'] (old -> new, time)."""
        b = await request.json()
        d = episode_path(DATA, b.get("id"))
        m = json.load(open(os.path.join(d, "meta.json")))
        if not m.get("t_end"):
            raise web.HTTPConflict(text="still recording")
        changes = {}
        for k, v in (b.get("changes") or {}).items():
            if k in EDITABLE and v is not None:
                v = EDITABLE[k](v)
                if m.get(k) != v:
                    changes[k] = dict(old=m.get(k), new=v)
                    m[k] = v
        if changes:
            m.setdefault("edit_log", []).append(dict(t=time.time(), source="console", changes=changes))
            tmp = os.path.join(d, "meta.json.tmp")
            json.dump(m, open(tmp, "w"), ensure_ascii=False, indent=1)
            os.replace(tmp, os.path.join(d, "meta.json"))
        return web.json_response(dict(ok=True, changed=sorted(changes)))

    async def episode_annotate(request):
        """post-hoc event marks from playback (2026-11-11: 接收方拿起物体 is marked afterwards on the third-person
        video). Kept in annotations.json next to the raw events.jsonl, which is never rewritten. One mark per kind;
        every set / move / delete is logged with the old and new time."""
        b = await request.json()
        d = episode_path(DATA, b.get("id"))
        kind, label = str(b.get("kind") or ""), str(b.get("label") or "")
        if not re.match(r"^[a-z_]{2,40}$", kind):
            raise web.HTTPBadRequest(text="bad kind")
        p = os.path.join(d, "annotations.json")
        try:
            a = json.load(open(p))
        except (OSError, ValueError):
            a = dict(events=[], log=[])
        old = next((e for e in a["events"] if e["kind"] == kind), None)
        a["events"] = [e for e in a["events"] if e["kind"] != kind]
        new = None
        if b.get("action") == "set":
            new = dict(kind=kind, label=label, t=float(b["t"]), source="post", at=time.time())
            a["events"].append(new)
            a["events"].sort(key=lambda e: e["t"])
        a["log"].append(dict(at=time.time(), action=b.get("action"), kind=kind,
                             old=old and old["t"], new=new and new["t"]))
        tmp = p + ".tmp"
        json.dump(a, open(tmp, "w"), ensure_ascii=False, indent=1)
        os.replace(tmp, p)
        return web.json_response(dict(ok=True, events=a["events"]))

    async def episode_trash(request):
        b = await request.json()
        d = episode_path(DATA, b.get("id"))
        if not json.load(open(os.path.join(d, "meta.json"))).get("t_end"):
            raise web.HTTPConflict(text="still recording")
        session, ep = d.split(os.sep)[-2:]
        dest = os.path.join(TRASH, session, f"{ep}__trashed_{time.strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        shutil.move(d, dest)
        try:
            os.rmdir(os.path.dirname(d))  # drop the session folder if it became empty
        except OSError:
            pass
        return web.json_response(dict(ok=True, trash_id=os.path.relpath(dest, TRASH)))

    async def trash_list(_):
        out = []
        for meta_path in glob.glob(os.path.join(TRASH, "*", "*", "meta.json")):
            d = os.path.dirname(meta_path)
            try:
                m = json.load(open(meta_path))
            except (OSError, ValueError):
                continue
            out.append(dict(id=os.path.relpath(d, TRASH), scenario=m.get("scenario"), episode=m.get("episode"),
                            version=m.get("version"), t_start=m.get("t_start"),
                            duration=(m["t_end"] - m["t_start"]) if m.get("t_end") else None,
                            mb=round(sum(os.path.getsize(os.path.join(d, f)) for f in os.listdir(d)) / 1e6, 1)))
        out.sort(key=lambda e: e["t_start"] or 0, reverse=True)
        return web.json_response(dict(episodes=out))

    async def trash_restore(request):
        b = await request.json()
        d = episode_path(TRASH, b.get("id"))
        session, ep = d.split(os.sep)[-2:]
        dest = os.path.join(DATA, session, ep.split("__trashed_")[0])
        if os.path.exists(dest):
            raise web.HTTPConflict(text="an episode with that name exists")
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        shutil.move(d, dest)
        try:
            os.rmdir(os.path.dirname(d))
        except OSError:
            pass
        return web.json_response(dict(ok=True))

    async def job_call(request, fn):
        try:
            b = await request.json()
            return web.json_response(dict(ok=True, result=fn(b)))
        except (ValueError, OSError, KeyError) as exc:
            return web.json_response(dict(ok=False, error=str(exc)), status=400)

    async def api_export(request):
        return await job_call(request, lambda b: jobs.export(b.get("name"), b.get("episodes") or [], b.get("arms", "left"),
                                                             b.get("cams") or ["head", "left", "right"],
                                                             bool(b.get("include_discarded"))))

    async def api_train(request):
        return await job_call(request, lambda b: jobs.train(b.get("dataset"), b.get("name"), b.get("steps", 20000),
                                                            b.get("batch", 8), b.get("cams") or []))

    async def api_cancel(request):
        return await job_call(request, lambda b: jobs.cancel(b.get("id")))

    async def index(_):
        return web.FileResponse(os.path.join(WEB, "index.html"), headers={"Cache-Control": "no-store"})

    @web.middleware
    async def revalidate(request, handler):  # page code changes often: browsers must revalidate (not reuse stale js)
        resp = await handler(request)
        if request.path.startswith("/web/") and "/vendor/" not in request.path:
            resp.headers["Cache-Control"] = "no-cache"
        return resp

    app = web.Application(middlewares=[revalidate])
    app.router.add_get("/", index)
    # phone remote (2026-11-11): e-stop and recording start/stop from the researcher's seat
    app.router.add_get("/m", lambda _: web.FileResponse(os.path.join(WEB, "remote.html"), headers={"Cache-Control": "no-store"}))
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/api/episodes", episodes)
    app.router.add_post("/api/episode/update", episode_update)
    app.router.add_post("/api/episode/trash", episode_trash)
    app.router.add_post("/api/episode/annotate", episode_annotate)
    app.router.add_get("/api/trash", trash_list)
    app.router.add_post("/api/trash/restore", trash_restore)
    app.router.add_post("/api/export", api_export)
    app.router.add_post("/api/train", api_train)
    app.router.add_post("/api/jobs/cancel", api_cancel)
    app.router.add_get("/api/jobs", lambda _: web.json_response(dict(jobs=jobs.list_jobs())))
    app.router.add_get("/api/datasets", lambda _: web.json_response(dict(datasets=jobs.datasets())))
    os.makedirs(DATA, exist_ok=True)
    app.router.add_static("/data/", DATA)  # episode files for playback (csv, jsonl, mp4)
    app.router.add_static("/web/", WEB)
    app.router.add_static("/urdf/", URDF_DIR)
    app.router.add_static("/mjcf/", MJCF_DIR)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, "0.0.0.0", a.port).start()
    print(f"console on http://0.0.0.0:{a.port} ({'mock' if a.mock else 'core'})", flush=True)
    await pump()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--mock", action="store_true", help="simulated joints for UI work without the robot core")
    asyncio.run(main_async(ap.parse_args()))
