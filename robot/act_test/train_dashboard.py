"""Training page for the console (ACT runs on the robot laptop). 2026-11-10 first version (user wanted to see the
whole training flow); 2026-11-11 rebuilt in the console's visual language and made live (user: the old page looked
static and unlike the main page; cancelling in the console left it showing "training").

Serves http://<robot>:8090 on the home LAN only (nothing leaves the machine). The page itself is dash/index.html,
dash.css, dash.js. Data per run comes from ~/fmp_train/runs/<name>/ (meta.json, train.log, gpu.csv, exit,
cancelled, eval.json, eval.log). API:
  GET  /api/runs            summary of every run (status, progress, rate) - polled every 2 s
  GET  /api/run?name=...    full series of one run (loss, step time, GPU, eval, log tail)
  POST /api/cancel {run}    cancel through the console's job system (console/jobs.py), same as the console button
Usage: train_dashboard.py [--port 8090]"""
import argparse, glob, json, os, re, sys, time
from urllib.parse import unquote
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

ROOT = os.path.expanduser("~/fmp_train")
HERE = os.path.dirname(os.path.abspath(__file__))
DASH = os.path.join(HERE, "dash")
sys.path.insert(0, os.path.join(HERE, "..", "console"))
LOG_FREQ = 25
STALE_S = 30  # no train.log / gpu.csv write for this long and no exit file: the run died without saying so
STEP_RE = re.compile(r"step:\S+ smpl:\S+ ep:\S+ epch:(\S+) (.*)")
TQDM_RE = re.compile(r"(\d+)/(\d+) \[([\d:]+)<([\d:?]+),\s*([\d.]+)(?:step|it)/s\]")
GPU_TOTAL_MIB = 6144
_cache = {}


def _mtime(p):
    try:
        return os.path.getmtime(p)
    except OSError:
        return None


def _num(v):
    try:
        x = float(v)
        return x if x == x and abs(x) != float("inf") else None
    except (TypeError, ValueError):
        return None


def _hms(s):
    try:
        parts = [int(x) for x in s.split(":")]
    except ValueError:
        return None
    t = 0
    for x in parts:
        t = t * 60 + x
    return t


def _parse(d):
    """Everything that needs the whole train.log, cached on the files' mtimes (finished runs are read once)."""
    files = ("train.log", "gpu.csv", "exit", "cancelled", "eval.json", "eval.log", "meta.json")
    key = tuple(_mtime(os.path.join(d, f)) for f in files)
    if d in _cache and _cache[d][0] == key:
        return _cache[d][1]
    meta = json.load(open(os.path.join(d, "meta.json")))
    st = dict(meta=meta, steps=[], loss=[], updt=[], data=[], epoch=None, gpu=[], error=None, exit=None, params=None,
              eval=None, tqdm=None, tail=[])
    try:
        raw = open(os.path.join(d, "train.log"), errors="replace").read()
    except OSError:
        raw = ""
    k, last_exc = 0, None
    for ln in raw.splitlines():
        m = STEP_RE.search(ln)
        if m:
            k += 1
            vals = dict(p.split(":", 1) for p in m.group(2).split() if ":" in p)
            st["steps"].append(k * LOG_FREQ)
            st["loss"].append(_num(vals.get("loss")))  # None, not NaN: NaN is not valid JSON for the browser
            st["updt"].append(_num(vals.get("updt_s")))
            st["data"].append(_num(vals.get("data_s")))
            st["epoch"] = float(m.group(1))
        if "num_learnable_params" in ln:
            st["params"] = ln.split("num_learnable_params=")[-1].split()[0].strip("()")
        if ("OutOfMemoryError" in ln or "CUDA out of memory" in ln) and not st["error"]:
            st["error"] = "显存不足（CUDA out of memory）：减少相机路数，或把批大小改为 4"
        elif re.match(r"\w*(Error|Exception):", ln):  # last exception line; the torchcodec fallback also logs one
            last_exc = ln
    tq = TQDM_RE.findall(raw[-4000:])
    if tq:
        cur, tot, el, rem, rate = tq[-1]
        st["tqdm"] = dict(step=int(cur), total=int(tot), elapsed=_hms(el), remaining=_hms(rem), rate=float(rate))
    # log tail for the page: real lines only (tqdm redraws with \r), newest last
    lines = [l.strip() for l in raw[-20000:].replace("\r", "\n").split("\n")]
    lines = [l for l in lines if len(l) > 12 and not l.startswith("Training:") and not re.search(r"/s\]|\d+/\d+ \[", l)]
    # "INFO 2026-11-10 22:46:09 t_train.py:1013 msg" -> "22:46:09  msg"
    st["tail"] = [re.sub(r"^(?:INFO|WARNING|ERROR) \d{4}-\d\d-\d\d (\d\d:\d\d:\d\d) \S+\.py:\d+ ", r"\1  ", l)[:220]
                  for l in lines[-4:]]
    if os.path.exists(os.path.join(d, "exit")):
        try:
            st["exit"] = int(open(os.path.join(d, "exit")).read().strip() or -1)
        except ValueError:
            st["exit"] = -1
        if st["exit"] != 0 and not st["error"]:
            st["error"] = "训练报错：" + (last_exc or "见 train.log")[:200]
    t0 = meta["start"]
    try:
        for ln in open(os.path.join(d, "gpu.csv")):
            p = [x.strip() for x in ln.split(",")]
            if len(p) >= 3:
                ts = time.mktime(time.strptime(p[0].split(".")[0], "%Y/%m/%d %H:%M:%S"))
                st["gpu"].append([round(ts - t0, 1), int(p[1]), int(p[2])])
    except (OSError, ValueError):
        pass
    st["gpu_peak"] = max((g[1] for g in st["gpu"]), default=0)
    if len(st["gpu"]) > 600:  # thin for the browser, keep the peak
        step = len(st["gpu"]) // 600 + 1
        peak = max(st["gpu"], key=lambda g: g[1])
        st["gpu"] = sorted(st["gpu"][::step] + [peak])
    e = os.path.join(d, "eval.json")
    if os.path.exists(e):
        try:
            st["eval"] = json.load(open(e))
        except ValueError:
            pass
    _cache[d] = (key, st)
    return st


def _phase(d, st):
    """starting / loading / training / evaluating / done / error / cancelled / interrupted, plus the end time."""
    now = time.time()
    if os.path.exists(os.path.join(d, "cancelled")):  # ended when the trainer stopped writing, not at the marker
        return "cancelled", max([t for t in (_mtime(os.path.join(d, f)) for f in ("train.log", "gpu.csv")) if t]
                                or [_mtime(os.path.join(d, "cancelled"))])
    if st["exit"] is not None:
        end = _mtime(os.path.join(d, "exit"))
        if st["exit"] != 0 or st["error"]:
            return "error", end
        ev = os.path.join(d, "eval.log")
        if st["eval"] is None and os.path.exists(ev) and now - (_mtime(ev) or 0) < 120:
            return "evaluating", now
        return "done", end
    last = max([t for t in (_mtime(os.path.join(d, f)) for f in ("train.log", "gpu.csv", "meta.json")) if t] or [now])
    if now - last > STALE_S:
        return "interrupted", last
    return ("training" if st["steps"] or st["tqdm"] else "loading"), now


def summary(d, full=False):
    st = _parse(d)
    phase, end = _phase(d, st)
    m = st["meta"]
    tq = st["tqdm"] or {}
    step = m["steps"] if phase in ("done", "evaluating") else max(tq.get("step", 0), st["steps"][-1] if st["steps"] else 0)
    out = dict(name=m["name"], label=m.get("label") or m["name"], repo=m.get("repo"), steps_total=m["steps"],
               batch=m.get("batch"), start=m["start"], phase=phase, step=step, elapsed=(end or time.time()) - m["start"],
               rate=tq.get("rate") if phase == "training" else None,
               remaining=tq.get("remaining") if phase == "training" else None,
               loss=next((x for x in reversed(st["loss"]) if x is not None), None), gpu_peak=st["gpu_peak"],
               error=st["error"], has_eval=st["eval"] is not None, epoch=st["epoch"])
    if phase == "interrupted" and not out["error"]:
        out["error"] = "训练进程已停止，但没有留下结束标记（可能是机载端重启或断电）"
    if full:
        out.update(series=dict(steps=st["steps"], loss=st["loss"], updt=st["updt"], data=st["data"], gpu=st["gpu"]),
                   eval=st["eval"], tail=st["tail"], params=st["params"], extra=m.get("extra"),
                   eval_failed=phase == "done" and st["eval"] is None and os.path.exists(os.path.join(d, "eval.log")))
    return out


def run_dirs():
    return sorted((r for r in glob.glob(os.path.join(ROOT, "runs", "*")) if os.path.exists(os.path.join(r, "meta.json"))),
                  key=lambda r: json.load(open(os.path.join(r, "meta.json"))).get("start", 0), reverse=True)


def cancel(run):
    import jobs  # console/jobs.py: the same path as the console's cancel button
    for j in jobs.list_jobs(limit=50):
        if j["kind"] == "train" and j.get("run") == run and j["state"] == "running":
            jobs.cancel(j["id"])
            return True
    raise ValueError("这条训练不是从控制台启动的，或已经结束")


CTYPES = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8", ".js": "text/javascript; charset=utf-8",
          ".jpg": "image/jpeg", ".json": "application/json"}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def js(self, obj, code=200):
        self.send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def file(self, f):
        if os.path.isfile(f):
            return self.send(200, open(f, "rb").read(), CTYPES.get(os.path.splitext(f)[1], "application/octet-stream"))
        self.send(404, b"not found", "text/plain")

    def do_GET(self):
        p, _, q = self.path.partition("?")
        if p in ("/", "/index.html"):
            return self.file(os.path.join(DASH, "index.html"))
        if p in ("/dash.css", "/dash.js"):
            return self.file(os.path.join(DASH, p[1:]))
        if p == "/api/runs":
            return self.js(dict(now=time.time(), gpu_total=GPU_TOTAL_MIB, runs=[summary(d) for d in run_dirs()]))
        if p == "/api/run":
            name = os.path.basename(unquote(dict(x.split("=", 1) for x in q.split("&") if "=" in x).get("name", "")))
            d = os.path.join(ROOT, "runs", name)
            if not name or not os.path.exists(os.path.join(d, "meta.json")):
                return self.js(dict(error="没有这条训练"), 404)
            return self.js(dict(now=time.time(), gpu_total=GPU_TOTAL_MIB, run=summary(d, full=True)))
        if p.startswith("/preview/"):
            return self.file(os.path.join(ROOT, "preview", os.path.basename(p)))
        self.send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/cancel":
            try:
                n = int(self.headers.get("Content-Length") or 0)
                cancel(json.loads(self.rfile.read(n) or b"{}").get("run", ""))
                return self.js(dict(ok=True))
            except Exception as e:  # noqa: BLE001 - shown to the user as is
                return self.js(dict(ok=False, error=str(e)), 400)
        self.send(404, b"not found", "text/plain")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    a = ap.parse_args()
    print(f"dashboard on http://0.0.0.0:{a.port}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()
