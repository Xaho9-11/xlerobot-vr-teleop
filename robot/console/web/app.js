// Console page: state stream -> UI + digital twin; buttons -> commands. console-architecture.md, 2026-11-10.
import { createTwin } from './twin.js';
import { createDataPanel } from './datapanel.js';

const $ = (s) => document.querySelector(s);
const MODES = [['idle', '空闲'], ['hold', '保持'], ['ready', '就绪'], ['teleop', '遥操作'], ['fold', '收回']];
const NEXT = { idle: ['torque_on', '上扭矩，保持当前位置'], hold: ['unfold', '展开到就绪姿态'], ready: ['start', '开始遥操作'],
  teleop: ['stop', '结束并收回'], replay: ['replay_stop', '停止回放（原地保持）'] };
const MOTION = { unfold: '展开中', fold: '收回中', replay_in: '移到起点' };
const GROUPS = [['头部', 'head_motor_'], ['左臂', 'left_arm_'], ['右臂', 'right_arm_']];
const NAMES = { head_motor_1: '水平', head_motor_2: '俯仰', shoulder_pan: '肩 · 旋转', shoulder_lift: '肩 · 抬升',
  elbow_flex: '肘', wrist_flex: '腕 · 俯仰', wrist_roll: '腕 · 翻转', gripper: '夹爪' };
const LINKS = [['bus1', '总线 1'], ['bus2', '总线 2'], ['video', '视频'], ['quest', '头显']];

let ws, limits = {}, rows = {}, twin = null, lit = null;

// scale the overlay with the screen; 1 = the design size (user's 16" screen), clamped for very small/large windows
const fitUI = () => {
  const s = Math.max(0.75, Math.min(2.2, Math.min(innerWidth / 1700, innerHeight / 1000)));
  document.documentElement.style.setProperty('--ui', s.toFixed(3));
};
addEventListener('resize', fitUI); fitUI();
$('#nav-train').href = `http://${location.hostname}:8090`;

const send = (cmd, arg) => ws && ws.readyState === 1 && ws.send(JSON.stringify({ type: 'cmd', cmd, arg }));
document.querySelectorAll('[data-cmd]').forEach((b) => b.addEventListener('click', () => send(b.dataset.cmd)));
$('#next').addEventListener('click', (e) => send(e.currentTarget.dataset.cmd));
$('#fold-set').addEventListener('click', () => {  // core fold_set (2026-11-13): fixed target of "stop and fold"
  if (confirm('把双臂和头部现在的姿态记录为折叠位？\n以后“停止并收回”都会回到这个姿态。')) send('fold_set');
});
$('#arms').addEventListener('click', (e) => e.target.dataset.v && send('arms', e.target.dataset.v));
$('#show-goal').addEventListener('change', (e) => twin && twin.showGoal(e.target.checked));

// ---- recording (core recorder + video_daemon cameras; right controller B also toggles)
let rec = null, lastEp = null;
$('#rec-btn').addEventListener('click', () => send(rec?.on ? 'rec_stop' : 'rec_start'));
const mark = (label) => rec?.on && send('rec_event', label);
document.querySelectorAll('[data-evt]').forEach((b) => b.addEventListener('click', () => mark(b.dataset.evt)));
addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  const b = document.querySelector(`#rec-evts button:nth-child(${e.key})`);
  if (b) mark(b.dataset.evt);
});
for (const k of ['participant', 'scenario', 'style']) {
  $(`#f-${k}`).addEventListener('change', (e) => send('rec_session', { [k]: e.target.value }));
}

// ---- target zone above the tray (core zone.py): automatic "arrived above target" (到达目标上方) event (2026-11-11)
$('#tz-set').addEventListener('click', () => {
  const z = lastState?.zone;
  if (z?.set && !confirm('用夹爪当前位置替换已记录的托盘位置？')) return;
  send('zone_set', lastState?.arms === 'right' ? 'right' : (z?.arm || 'left'));
});
$('#tz-r').addEventListener('change', (e) => send('zone_params', { radius: +e.target.value }));
function renderZone(z, r) {
  const st = $('#tz-st'), live = $('#tz-live');
  $('#tz').classList.toggle('in', !!z?.inside);
  if (!z) { st.textContent = '不可用'; live.textContent = ''; return; }
  if (!z.set) { st.textContent = '未设置'; live.textContent = '夹爪放到托盘中心、贴近表面后点“记录托盘位置”'; }
  else {
    st.textContent = `${z.arm === 'right' ? '右臂' : '左臂'} · 半径 ${Math.round(z.radius * 100)} cm`;
    const where = z.dist_cm == null ? '' : `距中心 ${z.dist_cm} cm · 高 ${z.height_cm} cm`;
    const got = r?.on ? (r.arrived != null ? `本条 ${fmtT(r.arrived)} 已到达` : '本条尚未到达') : '';
    live.innerHTML = `<i></i>${z.inside ? '在区域内' : '在区域外'}<span>${[where, got].filter(Boolean).join(' · ')}</span>`;
    const inp = $('#tz-r');
    if (document.activeElement !== inp) inp.value = Math.round(z.radius * 100);
  }
  $('#tz-r').disabled = !z.set;
  twin?.setZone?.(z);
}
const fmtT = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
function renderRec(r) {
  rec = r;
  if (!r) { $("#rec-btn").disabled = true; document.querySelectorAll("#rec-evts button").forEach((b) => { b.disabled = true; }); return; }
  const btn = $('#rec-btn');
  btn.classList.toggle('on', r.on);
  btn.disabled = false;
  $('#rec-label').textContent = r.on ? `录制中 · 第 ${r.episode} 条` : '开始录制';
  $('#rec-time').textContent = r.on ? fmtT(r.elapsed) : '';
  document.querySelectorAll('#rec-evts button').forEach((b) => { b.disabled = !r.on; });
  for (const k of ['participant', 'scenario', 'style']) {
    const el = $(`#f-${k}`);
    if (document.activeElement !== el) el.value = r.session?.[k] ?? '';
    el.disabled = r.on;
  }
  const m = r.on ? null : r.last;
  const key = m ? `${m.episode}|${m.final}|${m.repeat_of_final}|${m.discarded}|${m.reason}|${m.t_end}` : '';
  if (key === lastEp) return;  // only rebuild when the last episode changed (keeps the reason input editable)
  lastEp = key;
  if (!m) { $('#rec-last').innerHTML = ''; return; }
  const dur = m.t_end ? `${Math.round(m.t_end - m.t_start)} 秒` : '';
  $('#rec-last').innerHTML = `<div>上一条：第 ${m.episode} 条 · ${m.scenario} v${m.version} · ${dur}</div>
    <div class="chips">
      <button class="chip ${m.final ? 'on' : ''}" data-flag="final">最终版本</button>
      <button class="chip ${m.repeat_of_final ? 'on' : ''}" data-flag="repeat_of_final">最终版本的重复</button>
      <button class="chip bad ${m.discarded ? 'on' : ''}" data-flag="discarded">作废</button>
    </div>
    <input id="f-reason" placeholder="修改 / 重录理由" value="${(m.reason || '').replace(/"/g, '&quot;')}">`;
  $('#rec-last').querySelectorAll('[data-flag]').forEach((c) => c.addEventListener('click', () =>
    send('rec_annotate', { [c.dataset.flag]: !c.classList.contains('on') })));
  $('#f-reason').addEventListener('change', (e) => send('rec_annotate', { reason: e.target.value }));
}

// ---- data view (right rail): recordings, datasets, jobs - datapanel.js
let view = 'robot', dataPanel = null, playback = null, lastState = null;
function setView(v) {
  view = v;
  $('#nav-robot').classList.toggle('on', v === 'robot'); $('#nav-data').classList.toggle('on', v === 'data');
  $('#joints').hidden = v !== 'robot'; $('#data').hidden = v !== 'data';
  if (dataPanel) dataPanel.show(v === 'data');
}
$('#nav-robot').addEventListener('click', () => setView('robot'));
$('#nav-data').addEventListener('click', () => setView('data'));
if (location.hash === '#data') setView('data');  // the training page links back here (2026-11-11)
$('#pb-robot').addEventListener('click', () => {
  if (!playback?.active) return;
  if (confirm(`让机器人回放 #${playback.ep.episode} ${playback.ep.scenario} v${playback.ep.version}？
机器人会先移到这条录制的起始姿态，再按原速重放。请确认手臂周围没有人和障碍物。`)) {
    send('replay', playback.ep.id);
  }
});

function buildJoints(names) {
  const host = $('#joints'); host.innerHTML = '';
  for (const [label, prefix] of GROUPS) {
    const g = document.createElement('div'); g.className = 'group';
    g.innerHTML = `<h6>${label}</h6>`;
    for (const n of names.filter((x) => x.startsWith(prefix))) {
      const r = document.createElement('div'); r.className = 'joint';
      r.innerHTML = `<span class="n">${NAMES[n.slice(prefix.length)] || NAMES[n] || n}</span><span class="bar"><i class="g"></i><i class="p"></i></span><span class="v">—</span>`;
      r.addEventListener('pointerenter', () => setLit(n)); r.addEventListener('pointerleave', () => setLit(null));
      g.appendChild(r); rows[n] = r;
    }
    host.appendChild(g);
  }
}
function setLit(n) {
  if (lit && rows[lit]) rows[lit].classList.remove('lit');
  lit = n; if (n && rows[n]) rows[n].classList.add('lit');
  twin && twin.highlight(n);
}
const frac = (n, v) => { const [lo, hi] = limits[n] || [-180, 180]; return Math.max(0, Math.min(1, (v - lo) / (hi - lo))) * 100; };

function render(s) {
  const offline = s.mode === 'offline' || s.mode === 'nobus';
  $('#source').textContent = s.mock ? '模拟数据' : s.mode === 'nobus' ? '等待舵机总线' : offline ? '机器人核心未运行' : '机器人核心已连接';
  const idx = MODES.findIndex((m) => m[0] === (s.mode === 'replay' ? 'teleop' : s.mode));  // replay sits in the teleop slot
  $('#modes').innerHTML = MODES.map(([k, l], i) => {
    const base = s.mode === 'replay' && k === 'teleop' ? '回放中' : l;
    const label = i === idx && s.motion ? `${MOTION[s.motion.kind] || base} ${Math.round(s.motion.progress * 100)}%` : base;
    return `<li class="${i === idx ? 'on' : i < idx ? 'done' : ''}">${label}</li>`;
  }).join('');
  const [cmd, label] = s.motion && s.motion.kind !== 'replay_in' ? ['', '移动中…'] : s.mode === 'nobus' ? ['', '检查舵机电源与USB'] : offline ? ['', '等待机器人核心'] : (NEXT[s.mode] || ['', '']);
  const nb = $('#next'); nb.dataset.cmd = cmd; nb.textContent = label; nb.disabled = !cmd;
  document.querySelectorAll('[data-cmd="stop"]').forEach((b) => { b.disabled = !['hold', 'ready', 'teleop', 'replay'].includes(s.mode); });
  document.querySelectorAll('#arms button').forEach((b) => { b.classList.toggle('on', b.dataset.v === s.arms); b.disabled = s.mode !== 'idle'; });
  document.querySelectorAll('[data-cmd="recenter"]').forEach((b) => { b.disabled = s.mode !== 'teleop'; });
  $('#fold-set').hidden = !['idle', 'hold'].includes(s.mode);
  $('#quest').textContent = s.links.quest ? '已连接' : '未连接';
  $('#links').innerHTML = LINKS.map(([k, l]) => `<li><i class="dot ${s.links[k] ? 'ok' : 'bad'}"></i>${l}</li>`).join('');
  $('#prompts').innerHTML = (s.teleop?.hud || []).map((t) => `<div>${t}</div>`).join('');

  const meas = {}, goal = {};
  for (const [n, j] of Object.entries(s.joints)) {
    const r = rows[n];
    if (j.pos == null) { if (r) r.querySelector('.v').textContent = '—'; continue; }
    meas[n] = j.pos; goal[n] = j.goal ?? j.pos;
    if (!r) continue;
    r.classList.toggle('off', !j.torque);
    r.querySelector('.v').innerHTML = (n.endsWith('gripper') ? `${j.pos.toFixed(0)}%` : `${j.pos >= 0 ? '+' : '−'}${Math.abs(j.pos).toFixed(1)}°`)
      + (j.temp >= 55 ? ` <span class="warn">${j.temp}°C</span>` : '');
    r.querySelector('.p').style.left = `${frac(n, j.pos)}%`;
    r.querySelector('.g').style.left = `${frac(n, goal[n])}%`;
  }
  if (twin && !playback?.active) { twin.setMeasured(meas); twin.setGoal(goal); }
  lastState = s;
  const rb = $('#pb-robot');  // robot replay: only from the ready pose, with the recorded arms torqued
  rb.disabled = s.mode !== 'ready' || !!s.motion;
  rb.title = s.mode === 'ready' ? '' : '先上扭矩并展开到就绪姿态';

  renderRec(s.rec);
  if (!playback?.active) renderZone(s.zone, s.rec);
  else twin?.setZone?.(null);  // the zone belongs to the live pose, not to the replayed one
  if (s.rec && !s.rec.on && s.rec.last && s.rec.last.t_end !== window.__lastListed) { window.__lastListed = s.rec.last.t_end; dataPanel?.episodesChanged(); }
  const ev = s.events[s.events.length - 1];
  if (ev) $('#log').innerHTML = `${new Date(ev[0] * 1000).toLocaleTimeString('zh-CN', { hour12: false })}<b>${ev[1]}</b>`;
  const L = s.laptop || {};
  $('#sys').innerHTML = [
    L.ac != null && `电源 <b>${L.ac ? '市电' : '电池'}${L.battery != null ? ' ' + L.battery + '%' : ''}</b>`,
    L.gpu_temp != null && `GPU <b>${L.gpu_temp}°C</b>`,
    L.cpu_temp != null && `CPU <b>${L.cpu_temp}°C</b>`,
    L.disk_free_gb != null && `硬盘 <b>${L.disk_free_gb} GB</b>`,
  ].filter(Boolean).join('');
}

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.type === 'hello') { limits = m.limits; buildJoints(m.joints); }
    else if (m.type === 'state') render(m);
  };
  ws.onclose = () => { $('#source').textContent = '连接断开，重连中…'; setTimeout(connect, 1500); };
}

(async () => {
  connect();
  const map = await (await fetch('/web/twin_map.json')).json();
  const model = await (await fetch('/web/twin_model.json')).json();
  twin = await createTwin($('#twin'), { model, map, onHover: (n) => {
    if (lit && rows[lit]) rows[lit].classList.remove('lit');
    lit = n; if (n && rows[n]) rows[n].classList.add('lit');
  } });
  dataPanel = createDataPanel({ twin, send, host: $('#data') });
  playback = dataPanel.playback;
  if (view === 'data') dataPanel.show(true);
})();
