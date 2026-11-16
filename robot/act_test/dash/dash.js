// Training page (train_dashboard.py, rebuilt 2026-11-11). Polls /api/runs every 2 s; the selected run's series come
// from /api/run. Between polls the step counter, elapsed and remaining time advance locally (rate from the trainer's
// progress bar), so the page moves continuously; each poll corrects them. Only values and charts are redrawn.
'use strict';
const $ = (s, r = document) => r.querySelector(s);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
$('#nav-robot').href = `http://${location.hostname}:8080/`;
$('#nav-data').href = `http://${location.hostname}:8080/#data`;

const fitUI = () => {  // same rule as the console (console/web/app.js)
  const s = Math.max(0.75, Math.min(2.2, Math.min(innerWidth / 1700, innerHeight / 1000)));
  document.documentElement.style.setProperty('--ui', s.toFixed(3));
};
addEventListener('resize', () => { fitUI(); document.querySelectorAll('.plot').forEach(draw); });
fitUI();

const LIVE = new Set(['starting', 'loading', 'training', 'evaluating']);
const PHASE = {
  starting: ['启动中', 'run-on'], loading: ['读取数据', 'run-on'], training: ['训练中', 'run-on'],
  evaluating: ['离线检验中', 'run-on'], done: ['完成', 'ok'], error: ['出错', 'bad'], cancelled: ['已取消', ''],
  interrupted: ['已中断', 'bad'],
};
const STAGES = [['读取数据', '解码视频，标准化关节'], ['训练', '预测与示范对比，降低 loss'], ['保存模型', '写出检查点'],
  ['离线检验', '预测与示范动作对比'], ['上机运行', '第 2 步，手动']];
const C = { accent: '#86d3c8', warn: '#e2b45c', danger: '#ff5d4f', ink2: '#a2a8b1', ink3: '#5c636d' };
const JN = { shoulder_pan: '肩部水平', shoulder_lift: '肩部抬升', elbow_flex: '肘部', wrist_flex: '腕部俯仰',
  wrist_roll: '腕部翻转', gripper: '夹爪', head_motor_1: '头部 1', head_motor_2: '头部 2' };
const jname = (n) => {
  let side = '';
  if (n.startsWith('left_arm_')) { side = '左 · '; n = n.slice(9); } else if (n.startsWith('right_arm_')) { side = '右 · '; n = n.slice(10); }
  return side + (JN[n] || n);
};

const num = (n) => (n == null ? '—' : Math.round(n).toLocaleString('en-US'));
const clock = (s) => {
  if (s == null || !isFinite(s)) return '—';
  s = Math.max(0, Math.round(s));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = s % 60, p = (v) => String(v).padStart(2, '0');
  return h ? `${h}:${p(m)}:${p(x)}` : `${m}:${p(x)}`;
};
const date = (t) => { const d = new Date(t * 1000); return `${d.getMonth() + 1}月${d.getDate()}日 ${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}`; };
const lossFmt = (v) => (v == null ? '—' : v >= 100 ? v.toFixed(0) : v.toPrecision(3));

let runs = [], sel = new URLSearchParams(location.search).get('run'), detail = null, fetchedAt = 0, lastOk = 0,
  gpuTotal = 6144, preview, prevTail = [];

// ---------- polling
async function poll() {
  try {
    const j = await (await fetch('/api/runs', { cache: 'no-store' })).json();
    runs = j.runs; gpuTotal = j.gpu_total; lastOk = Date.now();
    if (!sel || !runs.some((r) => r.name === sel)) sel = (runs.find((r) => LIVE.has(r.phase)) || runs[0] || {}).name || null;
    renderList();
    const s = runs.find((r) => r.name === sel);
    if (!s) renderEmpty();
    else if (!detail || detail.name !== s.name || LIVE.has(s.phase) || detail.phase !== s.phase) await loadDetail(s.name);
  } catch (e) { /* shown by the live indicator */ }
  clearTimeout(poll.t); poll.t = setTimeout(poll, 2000);
}
async function loadDetail(name) {
  const j = await (await fetch('/api/run?name=' + encodeURIComponent(name), { cache: 'no-store' })).json();
  if (!j.run || j.run.name !== sel) return;  // the user picked another run meanwhile
  const first = !detail || detail.name !== j.run.name;
  detail = j.run; fetchedAt = Date.now(); gpuTotal = j.gpu_total;
  if (first) buildDetail();
  updateDetail();
}
function select(name) {
  if (name === sel) return;
  sel = name; detail = null;
  history.replaceState(null, '', '?run=' + encodeURIComponent(name));
  renderList(); loadDetail(name);
}

// ---------- run list
function renderList() {
  const host = $('#runs');
  if (!runs.length) { host.innerHTML = '<div class="empty">还没有训练。<br>在控制台「数据 → 数据集」里选择数据集开始训练。</div>'; return; }
  const names = runs.map((r) => r.name).join('|');
  if (host.dataset.names !== names) {
    host.dataset.names = names;
    host.innerHTML = runs.map((r) => `<button class="run" data-run="${esc(r.name)}"><div class="r1"><span class="nm"></span><span class="tag"></span></div>
      <div class="r2"></div><div class="rb"><i></i></div></button>`).join('');
    host.querySelectorAll('.run').forEach((b) => b.addEventListener('click', () => select(b.dataset.run)));
  }
  runs.forEach((r, i) => {
    const b = host.children[i], [lab, cls] = PHASE[r.phase] || [r.phase, ''];
    b.classList.toggle('on', r.name === sel);
    b.classList.toggle('live-run', LIVE.has(r.phase));
    b.classList.toggle('failed', r.phase === 'error' || r.phase === 'interrupted');
    $('.nm', b).textContent = r.name;
    const t = $('.tag', b); t.textContent = lab; t.className = 'tag ' + cls;
    $('.r2', b).textContent = `${date(r.start)} · ${num(r.steps_total)} 步 · 批 ${r.batch ?? '—'}`;
    $('.rb i', b).style.width = `${Math.min(100, (100 * r.step) / r.steps_total)}%`;
  });
}

// ---------- detail
function renderEmpty() {
  detail = null;
  $('#detail').innerHTML = '';
}
function fig(id, title, hint, cls = '') {
  return `<figure class="chart" id="${id}"><figcaption>${title}<span class="hint">${hint}</span><b class="val"></b></figcaption>
    <div class="plot ${cls}"></div></figure>`;
}
function buildDetail() {
  prevTail = [];
  $('#detail').innerHTML = `
    <div class="d-head"><div>
        <div class="d-title"><span id="d-name"></span><span class="tag" id="d-phase"></span></div>
        <div class="d-meta" id="d-meta"></div></div>
      <button class="txt danger" id="d-cancel" hidden>取消训练</button></div>
    <div class="prog">
      <div class="big"><b id="d-step">0</b><span>/ <span id="d-total"></span> 步</span><em id="d-pct"></em></div>
      <div class="pbar" id="d-bar"><i></i></div></div>
    <div class="stats">
      <div class="stat"><span>已用</span><b id="s-el">—</b></div>
      <div class="stat"><span>剩余</span><b id="s-rem">—</b></div>
      <div class="stat"><span>速度</span><b id="s-rate">—</b></div>
      <div class="stat"><span>当前 loss</span><b id="s-loss">—</b></div>
      <div class="stat"><span>显存峰值</span><b id="s-mem">—</b></div>
      <div class="stat"><span>数据遍数</span><b id="s-ep">—</b></div></div>
    <ol class="stages" id="d-stages">${STAGES.map((s) => `<li>${s[0]}<small>${s[1]}</small></li>`).join('')}</ol>
    <div class="err" id="d-err" hidden></div>
    <section class="sec"><h6>训练曲线</h6><div class="charts">
      ${fig('c-loss', 'loss', '对数坐标，越低越接近示范')}
      ${fig('c-mem', '显存', '红线为 6 GB 上限')}
      ${fig('c-time', '每步耗时', '<span class="legend"><span><i style="border-color:' + C.accent + '"></i>计算</span><span><i style="border-color:' + C.warn + '"></i>取数据</span></span>')}
      ${fig('c-util', 'GPU 利用率', '')}</div></section>
    <section class="sec"><h6>最新日志</h6><div class="log" id="d-log"></div></section>
    <section class="sec" id="d-eval" hidden></section>
    <section class="sec" id="d-preview" hidden></section>`;
  $('#d-cancel').addEventListener('click', cancelRun);
}

function updateDetail() {
  const r = detail, s = r.series, live = LIVE.has(r.phase), [lab, cls] = PHASE[r.phase] || [r.phase, ''];
  $('#d-name').textContent = r.name;
  const tag = $('#d-phase'); tag.textContent = lab; tag.className = 'tag ' + cls;
  $('#d-meta').textContent = [r.label, `${date(r.start)} 开始`, r.params ? `参数量 ${r.params}` : ''].filter(Boolean).join(' · ');
  $('#d-total').textContent = num(r.steps_total);
  $('#d-cancel').hidden = !live;
  const bar = $('#d-bar');
  bar.classList.toggle('moving', r.phase === 'training');
  bar.classList.toggle('stopped', r.phase === 'cancelled');
  bar.classList.toggle('failed', r.phase === 'error' || r.phase === 'interrupted');
  const err = $('#d-err');
  err.hidden = !r.error && !r.eval_failed;
  err.textContent = r.error || (r.eval_failed ? '训练完成，但离线检验没有生成结果，见 eval.log' : '');

  // stats that only change on a poll (the clock ones are in tick)
  const ls = s.loss.filter((v) => v != null), cur = ls[ls.length - 1], before = ls[Math.max(0, ls.length - 11)];
  const lossEl = $('#s-loss');
  lossEl.innerHTML = cur == null ? '—' : `${lossFmt(cur)}${ls.length > 10 ? ` <small>${cur < before ? '↓' : '↑'}</small>` : ''}`;
  lossEl.classList.toggle('down', ls.length > 10 && cur < before);
  $('#s-mem').innerHTML = r.gpu_peak ? `${(r.gpu_peak / 1024).toFixed(2)} <small>/ ${(gpuTotal / 1024).toFixed(0)} GB</small>` : '—';
  $('#s-ep').textContent = r.epoch == null ? '—' : `${r.epoch.toFixed(2)} 遍`;

  // pipeline
  const reached = r.step > 0 || s.steps.length ? 1 : 0;
  const at = { starting: 0, loading: 0, training: 1, evaluating: 3, done: 5 }[r.phase] ?? reached;
  document.querySelectorAll('#d-stages li').forEach((li, i) => {
    let c = i < at ? 'done' : i === at ? 'on' : '';
    if (r.phase === 'done') c = i < 4 ? 'done' : '';
    if (r.phase === 'done' && i === 3 && !r.has_eval) c = r.eval_failed ? 'bad' : '';
    if (['error', 'interrupted', 'cancelled'].includes(r.phase) && i === at) c = r.phase === 'cancelled' ? 'stop' : 'bad';
    li.className = c;
  });

  // charts
  const train = r.phase === 'training';
  const lossXY = s.steps.map((x, i) => [x, s.loss[i]]);
  setChart('#c-loss', {
    series: [{ xy: lossXY, color: C.accent, area: true }], log: true, x0: 0, x1: r.steps_total, live: train,
    fx: num, val: cur == null ? '' : `当前 ${lossFmt(cur)}`,
    hover: (i) => `第 ${num(s.steps[i])} 步 · ${lossFmt(s.loss[i])}`, wait: r.phase === 'loading' ? '读取数据、创建模型中…' : '等待数据…',
  });
  const memXY = s.gpu.map((g) => [g[0], g[1] / 1024]);
  setChart('#c-mem', {
    series: [{ xy: memXY, color: C.accent, area: true }], y0: 0, hline: gpuTotal / 1024, x0: 0, live,
    fx: clock, fy: (v) => v.toFixed(1), val: r.gpu_peak ? `峰值 ${(r.gpu_peak / 1024).toFixed(2)} GB` : '',
    hover: (i) => `${clock(s.gpu[i][0])} · ${(s.gpu[i][1] / 1024).toFixed(2)} GB`,
  });
  const n = s.steps.length;
  setChart('#c-time', {
    series: [{ xy: s.steps.map((x, i) => [x, s.updt[i]]), color: C.accent }, { xy: s.steps.map((x, i) => [x, s.data[i]]), color: C.warn }],
    y0: 0, x0: 0, x1: r.steps_total, live: train, fx: num, fy: (v) => v.toFixed(2),
    val: n ? `${s.updt[n - 1]?.toFixed(3)} s + ${s.data[n - 1]?.toFixed(3)} s` : '',
    hover: (i) => `第 ${num(s.steps[i])} 步 · 计算 ${s.updt[i]?.toFixed(3)} s · 取数据 ${s.data[i]?.toFixed(3)} s`,
  });
  const util = s.gpu.map((g) => [g[0], g[2]]), u = s.gpu.length ? s.gpu[s.gpu.length - 1][2] : null;
  const avg = s.gpu.length ? s.gpu.reduce((a, g) => a + g[2], 0) / s.gpu.length : null;
  setChart('#c-util', {
    series: [{ xy: util, color: C.accent, area: true }], y0: 0, y1: 100, x0: 0, live, fx: clock, fy: (v) => v.toFixed(0),
    val: u == null ? '' : live ? `当前 ${u}%` : `平均 ${avg.toFixed(0)}%`,
    hover: (i) => `${clock(s.gpu[i][0])} · ${s.gpu[i][2]}%`,
  });

  // log tail: only new lines animate
  const tail = r.tail || [];
  if (tail.join('\n') !== prevTail.join('\n')) {
    $('#d-log').innerHTML = tail.map((l) => `<div${prevTail.includes(l) ? ' style="animation:none"' : ''}>${esc(l)}</div>`).join('') || '<div>—</div>';
    prevTail = tail;
  }
  renderEval(r);
  renderPreview(r);
  tick();
}

// clock values advance between polls
function tick() {
  if (!detail) return;
  const r = detail, dt = (Date.now() - fetchedAt) / 1000, train = r.phase === 'training', live = LIVE.has(r.phase);
  const step = train && r.rate ? Math.min(r.steps_total, r.step + r.rate * dt) : r.step;
  const frac = Math.min(1, step / r.steps_total);
  $('#d-step').textContent = num(Math.floor(step));
  $('#d-pct').textContent = `${(frac * 100).toFixed(frac < 0.1 ? 1 : 0)}%`;
  $('#d-bar i').style.width = `${frac * 100}%`;
  $('#s-el').textContent = clock(r.elapsed + (live ? dt : 0));
  $('#s-rem').textContent = train && r.remaining != null ? clock(r.remaining - dt) : '—';
  $('#s-rate').innerHTML = train && r.rate ? `${r.rate.toFixed(2)} <small>步/秒</small>`
    : r.step && r.elapsed ? `${(r.step / r.elapsed).toFixed(2)} <small>步/秒 · 平均</small>` : '—';
}
setInterval(tick, 200);

// live indicator
setInterval(() => {
  const el = $('#live'), ago = (Date.now() - lastOk) / 1000;
  el.className = 'live ' + (!lastOk ? '' : ago > 6 ? 'bad' : 'ok');
  $('span', el).textContent = !lastOk ? '连接中…' : ago > 6 ? '连接中断，重试中' : `实时同步 · ${Math.max(0, Math.round(ago))} 秒前`;
}, 500);

async function cancelRun() {
  if (!detail || !confirm(`取消训练 ${detail.name}？\n还没保存的进度会丢失，显卡会立即释放。`)) return;
  const b = $('#d-cancel'); b.disabled = true; b.textContent = '正在取消…';
  try {
    const j = await (await fetch('/api/cancel', { method: 'POST', body: JSON.stringify({ run: detail.name }) })).json();
    if (!j.ok) alert(j.error || '取消失败');
  } catch (e) { alert('取消失败：' + e); }
  b.disabled = false; b.textContent = '取消训练';
  poll();
}

// ---------- eval and dataset preview (drawn once per run)
function renderEval(r) {
  const host = $('#d-eval');
  if (!r.eval) { host.hidden = true; host.dataset.run = ''; return; }
  if (host.dataset.run === r.name) return;
  host.dataset.run = r.name; host.hidden = false;
  const e = r.eval, xs = e.pred.map((_, i) => i / e.fps);
  host.innerHTML = `<h6>离线检验<span class="hint">平均误差 ${e.mae.toFixed(2)}°</span></h6>
    <p class="note">第 ${e.episode} 条演示，从第 ${e.start} 帧起，模型一次预测 ${e.pred.length} 步动作，与示范中真实的动作对比；两条线越重合越好。</p>
    <div class="legend" style="margin-bottom:14px"><span><i style="border-color:${C.ink2}"></i>示范</span><span><i class="dash" style="border-color:${C.accent}"></i>模型预测</span></div>
    <div class="evals">${e.names.map((n, j) => fig('ev-' + j, jname(n), '', 'small')).join('')}</div>`;
  e.names.forEach((n, j) => setChart('#ev-' + j, {
    series: [{ xy: e.true.map((v, i) => [xs[i], v[j]]), color: C.ink2 }, { xy: e.pred.map((v, i) => [xs[i], v[j]]), color: C.accent, dash: true }],
    fx: (v) => `${v.toFixed(1)} s`, fy: (v) => v.toFixed(0), val: '',
    hover: (i) => `示范 ${e.true[i][j].toFixed(1)}° · 预测 ${e.pred[i][j].toFixed(1)}°`,
  }));
}
async function renderPreview(r) {
  const host = $('#d-preview');
  if (preview === undefined) preview = await fetch('/preview/preview.json').then((x) => (x.ok ? x.json() : null)).catch(() => null);
  if (!preview || r.repo !== preview.repo) { host.hidden = true; host.dataset.run = ''; return; }
  if (host.dataset.run === r.name) return;
  host.dataset.run = r.name; host.hidden = false;
  const p = preview, cams = { top: '顶部相机', wrist: '腕部相机' };
  host.innerHTML = `<h6>训练数据<span class="hint">${esc(p.repo)} · ${p.episodes} 条演示 · ${num(p.frames_total)} 帧 · ${p.fps} fps</span></h6>
    <p class="note">任务：${esc(p.task)}</p>
    ${p.cams.map((c) => `<div class="frames">${p.frames.map((f) => `<figure><img src="/preview/${esc(f.imgs[c])}" loading="lazy"><figcaption>${cams[c] || c} · ${f.t} s</figcaption></figure>`).join('')}</div>`).join('')}
    ${fig('pv-joints', `第 ${p.episode} 条演示的关节角`, `${p.length} 帧`)}`;
  const pal = [C.accent, C.warn, C.ink2, '#9aa7ff', '#d79bd0', C.danger];
  setChart('#pv-joints', {
    series: p.joint_names.map((n, j) => ({ xy: p.state.map((s, i) => [i / p.fps, s[j]]), color: pal[j % pal.length] })),
    fx: (v) => `${v.toFixed(0)} s`, fy: (v) => v.toFixed(0), val: '',
    hover: (i) => p.joint_names.map((n, j) => `${JN[n] || n} ${p.state[i][j].toFixed(0)}°`).join(' · '),
  });
}

// ---------- charts: SVG sized to the box (1 unit = 1 px), hover cursor, pulsing end point while live
function setChart(sel, cfg) {
  const fig = $(sel); if (!fig) return;
  const plot = $('.plot', fig);
  plot._cfg = cfg; plot._fig = fig;
  if (!plot._wired) {
    plot._wired = true;
    plot.addEventListener('mousemove', (ev) => hover(plot, ev));
    plot.addEventListener('mouseleave', () => { plot._hi = null; draw(plot); });
  }
  draw(plot);
}
function draw(plot) {
  const c = plot._cfg; if (!c) return;
  const W = plot.clientWidth, H = plot.clientHeight; if (!W || !H) return;
  const val = $('.val', plot._fig);
  const pts = c.series.flatMap((s) => s.xy.filter((p) => p[1] != null && isFinite(p[1])));
  if (!pts.length) { plot.innerHTML = `<div class="wait">${c.wait || '等待数据…'}</div>`; val.textContent = ''; plot._geo = null; return; }
  const f = (v) => (c.log ? Math.log10(Math.max(v, 1e-6)) : v);
  let x0 = c.x0 ?? Math.min(...pts.map((p) => p[0])), x1 = Math.max(c.x1 ?? -Infinity, ...pts.map((p) => p[0]));
  let y0 = c.y0 != null ? f(c.y0) : Math.min(...pts.map((p) => f(p[1]))), y1 = c.y1 != null ? f(c.y1) : Math.max(...pts.map((p) => f(p[1])));
  if (c.hline != null) y1 = Math.max(y1, f(c.hline) * 1.04);
  if (c.y1 == null && c.y0 == null) { const pad = (y1 - y0) * 0.08 || 1; y0 -= pad; y1 += pad; } else if (c.y1 == null) y1 += (y1 - y0) * 0.08 || 1;
  if (x1 === x0) x1 = x0 + 1;
  const L = 44, R = 8, T = 6, B = 20, X = (v) => L + ((v - x0) / (x1 - x0)) * (W - L - R), Y = (v) => T + (1 - (f(v) - y0) / (y1 - y0)) * (H - T - B);
  const fy = c.fy || ((v) => (+v.toPrecision(3)).toString());
  let g = '';
  for (let i = 0; i <= 3; i++) {
    const yv = y0 + ((y1 - y0) * i) / 3, yy = T + (1 - i / 3) * (H - T - B);
    g += `<line x1="${L}" x2="${W - R}" y1="${yy}" y2="${yy}" stroke="rgba(255,255,255,${i ? 0.05 : 0.12})"/>
      <text x="${L - 8}" y="${yy + 4}" text-anchor="end">${fy(c.log ? 10 ** yv : yv)}</text>`;
  }
  const fx = c.fx || ((v) => v.toFixed(0));
  g += `<text x="${L}" y="${H - 4}">${fx(x0)}</text><text x="${W - R}" y="${H - 4}" text-anchor="end">${fx(x1)}</text>`;
  if (c.hline != null) {
    g += `<line x1="${L}" x2="${W - R}" y1="${Y(c.hline)}" y2="${Y(c.hline)}" stroke="${C.danger}" stroke-opacity=".6" stroke-dasharray="3 4"/>`;
  }
  const id = 'g' + Math.random().toString(36).slice(2, 8);
  c.series.forEach((s, k) => {
    const p = s.xy.filter((q) => q[1] != null && isFinite(q[1]));
    if (!p.length) return;
    const line = p.map((q) => `${X(q[0]).toFixed(1)},${Y(q[1]).toFixed(1)}`).join(' ');
    if (s.area) {
      g += `<defs><linearGradient id="${id}${k}" x1="0" x2="0" y1="0" y2="1"><stop offset="0" stop-color="${s.color}" stop-opacity=".18"/>
        <stop offset="1" stop-color="${s.color}" stop-opacity="0"/></linearGradient></defs>
        <polygon fill="url(#${id}${k})" points="${X(p[0][0]).toFixed(1)},${H - B} ${line} ${X(p[p.length - 1][0]).toFixed(1)},${H - B}"/>`;
    }
    g += `<polyline fill="none" stroke="${s.color}" stroke-width="1.5" stroke-linejoin="round" ${s.dash ? 'stroke-dasharray="4 3"' : ''} points="${line}"/>`;
    if (c.live && k === 0) {
      const e = p[p.length - 1];
      g += `<circle class="pulse" cx="${X(e[0])}" cy="${Y(e[1])}" r="3" fill="${s.color}"/><circle cx="${X(e[0])}" cy="${Y(e[1])}" r="3" fill="${s.color}"/>`;
    }
  });
  if (plot._hi != null) {
    const i = plot._hi, p0 = c.series[0].xy[i];
    if (p0) {
      g += `<line x1="${X(p0[0])}" x2="${X(p0[0])}" y1="${T}" y2="${H - B}" stroke="rgba(255,255,255,.25)"/>`;
      c.series.forEach((s) => { const q = s.xy[i]; if (q && q[1] != null) g += `<circle cx="${X(q[0])}" cy="${Y(q[1])}" r="3" fill="#0a0c0f" stroke="${s.color}" stroke-width="1.5"/>`; });
    }
  }
  plot.innerHTML = `<svg viewBox="0 0 ${W} ${H}">${g}</svg>`;
  plot._geo = { L, R, W, x0, x1 };
  val.textContent = plot._hi != null && c.hover ? c.hover(plot._hi) : c.val || '';
  val.style.color = plot._hi != null ? 'var(--accent)' : '';
}
function hover(plot, ev) {
  const g = plot._geo, c = plot._cfg; if (!g || !c) return;
  const rect = plot.getBoundingClientRect(), px = ((ev.clientX - rect.left) * g.W) / rect.width;
  const xv = g.x0 + ((px - g.L) / (g.W - g.L - g.R)) * (g.x1 - g.x0), xy = c.series[0].xy;
  if (!xy.length) return;
  let lo = 0, hi = xy.length - 1;  // x is sorted: binary search for the nearest sample
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xy[m][0] < xv) lo = m; else hi = m; }
  const i = Math.abs(xy[lo][0] - xv) <= Math.abs(xy[hi][0] - xv) ? lo : hi;
  if (i !== plot._hi) { plot._hi = i; draw(plot); }
}

poll();
