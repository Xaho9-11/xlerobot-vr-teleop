// Episode playback in the digital twin (2026-11-10): recorded joints drive the model (solid = measured, translucent
// = commanded), the three camera videos follow the same clock, events are ticks on the timeline. Read-only: it never
// talks to the robot (robot replay is a separate core command).
const $ = (s) => document.querySelector(s);

function parseCSV(text) {
  const lines = text.trim().split('\n');
  const head = lines[0].split(',');
  return { head, rows: lines.slice(1).map((l) => l.split(',')) };
}
const parseCamTimes = (text) => text.trim().split('\n').slice(1).map((l) => +l.split(',')[1]);
const fmt = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${(s % 60).toFixed(1).padStart(4, '0')}`;
// wall-clock time of a recorded moment (user request 2026-11-10: materials carry their real recording time)
export const fmtDate = (t) => new Date(t * 1000).toLocaleDateString('zh-CN', { year: 'numeric', month: '2-digit', day: '2-digit' });
export const fmtClock = (t, tenths = false) => {
  const d = new Date(t * 1000);
  const s = d.toLocaleTimeString('zh-CN', { hour12: false });
  return tenths ? `${s}.${Math.floor((d.getMilliseconds()) / 100)}` : s;
};

export class Playback {
  constructor({ twin, onClose }) {
    this.twin = twin; this.onClose = onClose;
    this.active = false; this.playing = false; this.t = 0; this.speed = 1;
    $('#pb-play').addEventListener('click', () => this.toggle());
    $('#pb-close').addEventListener('click', () => this.close());
    $('#pb-speed').addEventListener('click', () => { this.speed = this.speed === 1 ? 0.5 : this.speed === 0.5 ? 2 : 1; $('#pb-speed').textContent = `${this.speed}×`; });
    const bar = $('#pb-bar');
    const seek = (e) => { const r = bar.getBoundingClientRect(); this.seek(((e.clientX - r.left) / r.width) * this.T); };
    bar.addEventListener('pointerdown', (e) => { bar.setPointerCapture(e.pointerId); seek(e); bar.onpointermove = seek; });
    bar.addEventListener('pointerup', () => { bar.onpointermove = null; });
    addEventListener('keydown', (e) => {
      if (!this.active || /INPUT|TEXTAREA/.test(e.target.tagName)) return;
      if (e.code === 'Space') { e.preventDefault(); this.toggle(); }
      // frame stepping for post-hoc marks (2026-11-11): arrows = one frame, Shift + arrows = one second
      if (e.code === 'ArrowLeft' || e.code === 'ArrowRight') {
        e.preventDefault();
        this.step((e.code === 'ArrowLeft' ? -1 : 1) * (e.shiftKey ? 1 : 1 / 30));
      }
    });
    const loop = (now) => {
      if (this.active && this.playing) {
        const dt = this.last ? (now - this.last) / 1000 : 0;
        this.seek(this.t + dt * this.speed, true);
        if (this.t >= this.T) this.pause();
      }
      this.last = now;
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  async open(ep) {
    const base = `/data/${ep.id}/`;
    const get = (f) => fetch(base + f, { cache: 'no-store' }).then((r) => (r.ok ? r.text() : ''));
    const [robot, events, annots, ...cams] = await Promise.all([get('robot.csv'), get('events.jsonl'), get('annotations.json'),
      ...ep.cams.map((c) => get(`cam_${c}.csv`))]);
    try { this.annots = annots ? JSON.parse(annots).events || [] : []; } catch (e) { this.annots = []; }
    const { head, rows } = parseCSV(robot);
    if (!rows.length) return false;
    const col = {}; head.forEach((h, i) => { col[h] = i; });
    this.joints = [...new Set(head.filter((h) => h.endsWith('.pos')).map((h) => h.slice(0, -4)))];
    this.ts = rows.map((r) => +r[0]);
    this.t0 = this.ts[0]; this.T = this.ts[this.ts.length - 1] - this.t0;
    this.pos = rows.map((r) => Object.fromEntries(this.joints.map((j) => [j, r[col[`${j}.pos`]] === '' ? null : +r[col[`${j}.pos`]]])));
    this.goal = rows.map((r) => Object.fromEntries(this.joints.map((j) => [j, r[col[`${j}.goal`]] === '' ? null : +r[col[`${j}.goal`]]])));
    this.events = events.trim() ? events.trim().split('\n').map((l) => JSON.parse(l)) : [];
    // videos: <video> time 0 = the camera's first captured frame
    this.videos = ep.cams.map((c, k) => {
      const ts = parseCamTimes(cams[k] || '');
      const v = document.createElement('video');
      v.src = base + `cam_${c}.mp4`; v.muted = true; v.preload = 'auto'; v.playsInline = true;
      return { name: c, el: v, start: ts[0] ?? this.t0 };
    });
    this.ep = ep; this.active = true; this.t = 0; this.playing = false;
    this.renderBar();
    $('#player').hidden = false; document.body.classList.add('playback');
    this.seek(0);
    return true;
  }

  close() {
    this.pause(); this.active = false;
    for (const v of this.videos || []) v.el.src = '';
    $('#player').hidden = true; document.body.classList.remove('playback', 'pb-big');
    this.grid?.querySelectorAll('figure').forEach((x) => x.classList.remove('big'));
    this.onClose && this.onClose();
  }

  toggle() { this.playing ? this.pause() : this.play(); }
  step(dt) { this.pause(); this.seek(this.t + dt); }

  // recorded events (events.jsonl, never rewritten) + post-hoc marks (annotations.json), in time order
  marks() {
    return [...this.events.filter((e) => !['episode_start', 'episode_end'].includes(e.kind)), ...(this.annots || [])]
      .sort((a, b) => a.t - b.t);
  }
  setAnnotations(list) { this.annots = list; this.renderBar(); this.seek(this.t); }
  play() {
    if (this.t >= this.T) this.seek(0);
    this.playing = true; $('#pb-play').textContent = '❚❚';
    for (const v of this.videos) { v.el.playbackRate = this.speed; v.el.play().catch(() => {}); }
  }
  pause() { this.playing = false; $('#pb-play').textContent = '▶'; for (const v of this.videos || []) v.el.pause(); }

  index(t) {  // last row at or before t (binary search)
    const x = this.t0 + t; let lo = 0, hi = this.ts.length - 1;
    while (lo < hi) { const m = (lo + hi + 1) >> 1; if (this.ts[m] <= x) lo = m; else hi = m - 1; }
    return lo;
  }

  seek(t, fromLoop = false) {
    this.t = Math.max(0, Math.min(this.T, t));
    const i = this.index(this.t);
    this.twin.setMeasured(Object.fromEntries(Object.entries(this.pos[i]).filter(([, v]) => v != null)));
    this.twin.setGoal(Object.fromEntries(Object.entries(this.goal[i]).filter(([, v]) => v != null)));
    for (const v of this.videos) {  // keep videos on the shared clock; only nudge when they drift (or on a seek)
      const want = Math.max(0, this.t0 + this.t - v.start);
      if (!fromLoop || Math.abs(v.el.currentTime - want) > 0.25) v.el.currentTime = want;
      v.el.playbackRate = this.speed;
    }
    $('#pb-fill').style.width = `${(this.t / this.T) * 100}%`;
    $('#pb-time').textContent = `${fmt(this.t)} / ${fmt(this.T)}`;
    $('#pb-clock').textContent = fmtClock(this.t0 + this.t, true);
    const now = [...this.marks()].reverse().find((e) => e.t - this.t0 <= this.t);
    $('#pb-event').textContent = now ? now.label || now.kind : '';
    this.onSeek && this.onSeek(this.t);
  }

  renderBar() {
    $('#pb-ticks').innerHTML = this.marks().map((e) =>
      `<i class="${e.source === 'researcher' ? 'man' : e.source === 'post' ? 'post' : ''}" style="left:${((e.t - this.t0) / this.T) * 100}%" title="${e.label || e.kind}"></i>`).join('');
  }

  // the camera videos as one element; the same <video> elements are reused, so the data panel can re-render
  // around them without interrupting playback
  videoGrid() {
    if (!this.grid) {
      this.grid = document.createElement('div'); this.grid.className = 'pb-videos';
    }
    this.grid.innerHTML = '';
    for (const v of this.videos || []) {
      const f = document.createElement('figure');
      f.title = '点击放大 / 还原';
      f.addEventListener('click', () => {  // enlarge one camera over the scene (judging a grasp needs detail)
        const big = !f.classList.contains('big');
        this.grid.querySelectorAll('figure').forEach((x) => x.classList.remove('big'));
        f.classList.toggle('big', big);
        document.body.classList.toggle('pb-big', big);
      });
      f.appendChild(v.el);
      const cap = document.createElement('figcaption');
      cap.textContent = { head: '头部相机', left: '左腕相机', right: '右腕相机', scene: '第三人称' }[v.name] || v.name;
      f.appendChild(cap); this.grid.appendChild(f);
    }
    return this.grid;
  }
}
