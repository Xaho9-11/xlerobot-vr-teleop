// Right-rail data panel (redesigned 2026-11-10 after user feedback: list, trash and datasets were one long mixed
// list with inconsistent type and controls). Structure: three tabs (recordings / datasets / jobs); one page at a time - a
// list, or a detail page reached from it with "← back". Every list uses the same two-line row (title + right value,
// grey meta line), the same state tags, and three kinds of button (primary / secondary / text).
import { Playback, fmtDate, fmtClock } from './playback.js';

const $ = (s, r = document) => r.querySelector(s);
const esc = (t) => String(t ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const post = (url, body) => fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
  .then(async (r) => { const j = await r.json().catch(() => ({})); if (!r.ok || j.ok === false) throw new Error(j.error || r.statusText); return j; });
const get = (url) => fetch(url, { cache: 'no-store' }).then((r) => r.json());
const mmss = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
const CAM = { head: '头部', left: '左腕', right: '右腕' };
const ARMS = { left: '左臂', right: '右臂', both: '双臂' };
const JOB = { running: ['进行中', 'accent'], done: ['完成', 'muted'], failed: ['失败', 'danger'], cancelled: ['已取消', 'muted'] };
const sessionLabel = (s) => { const m = /^(\d{4})(\d{2})(\d{2})_(.+)$/.exec(s); return m ? `${+m[2]}月${+m[3]}日 · ${m[4]}` : s; };
const epTitle = (e) => `#${String(e.episode).padStart(3, '0')} ${esc(e.scenario)}<span class="ver">v${e.version}</span>`;
const tags = (e) => [e.final && '<i class="tag accent">最终</i>', e.repeat_of_final && '<i class="tag">重复</i>',
  e.discarded && '<i class="tag danger">作废</i>'].filter(Boolean).join('');

export function createDataPanel({ twin, send, host }) {
  const st = { tab: 'rec', page: null, episodes: [], free: null, trash: [], datasets: [], jobs: [], sel: new Set(), msg: '' };
  const playback = new Playback({ twin, onClose: () => { if (st.page?.type === 'episode') { st.page = null; render(); } } });
  let visible = false, timer = null;

  // ---------------------------------------------------------------- data
  async function load(what = ['episodes', 'datasets', 'jobs']) {
    const jobs = [];
    if (what.includes('episodes')) jobs.push(get('/api/episodes').then((r) => { st.episodes = r.episodes; st.free = r.free_gb; }));
    if (what.includes('trash')) jobs.push(get('/api/trash').then((r) => { st.trash = r.episodes; }));
    if (what.includes('datasets')) jobs.push(get('/api/datasets').then((r) => { st.datasets = r.datasets; }));
    if (what.includes('jobs')) jobs.push(get('/api/jobs').then((r) => { st.jobs = r.jobs; }));
    await Promise.all(jobs);
  }
  function poll() {  // jobs while the panel is open: fast while something runs
    clearTimeout(timer);
    if (!visible) return;
    timer = setTimeout(async () => {
      const before = st.jobs.filter((j) => j.state === 'running').length;
      await load(['jobs']);
      const running = st.jobs.filter((j) => j.state === 'running').length;
      if (running !== before) await load(['datasets']);  // an export may have finished
      if (!isTyping()) render();
      poll();
    }, st.jobs.some((j) => j.state === 'running') ? 2000 : 8000);
  }
  const isTyping = () => host.contains(document.activeElement) && /INPUT|TEXTAREA/.test(document.activeElement.tagName);

  // ---------------------------------------------------------------- navigation
  function go(page) {
    if (st.page?.type === 'episode' && page?.type !== 'episode') playback.close();
    st.page = page; st.msg = '';
    render();
    if (page?.type === 'episode') {
      const e = st.episodes.find((x) => x.id === page.id);
      if (e) playback.open(e).then((ok) => { const ph = $('#pb-videos', host); if (ok && ph) ph.replaceWith(playback.videoGrid()); if (ok) renderMarks(); });
    }
    host.scrollTop = 0;
  }
  function tab(t) { st.tab = t; go(null); }

  // ---------------------------------------------------------------- render
  function render() {
    const running = st.jobs.filter((j) => j.state === 'running').length;
    const tabs = `<nav class="dp-tabs">${[['rec', '录制'], ['ds', '数据集'], ['jobs', '任务']].map(([k, l]) =>
      `<button data-tab="${k}" class="${st.tab === k ? 'on' : ''}">${l}${k === 'jobs' && running ? `<i class="count">${running}</i>` : ''}</button>`).join('')}</nav>`;
    const p = st.page;
    let body;
    if (p?.type === 'episode') body = pageEpisode(st.episodes.find((x) => x.id === p.id));
    else if (p?.type === 'trash') body = pageTrash();
    else if (p?.type === 'export') body = pageExport();
    else if (p?.type === 'dataset') body = pageDataset(st.datasets.find((x) => x.name === p.name));
    else body = { rec: listEpisodes, ds: listDatasets, jobs: listJobs }[st.tab]();
    host.innerHTML = (p ? '' : tabs) + body;
    if (p?.type === 'episode' && playback.active) { $('#pb-videos', host).replaceWith(playback.videoGrid()); renderMarks(); }
  }

  const back = (label) => `<button class="txt back" data-go="back">← ${label}</button>`;

  function listEpisodes() {
    const done = st.episodes.filter((e) => e.duration != null);
    const groups = {};
    for (const e of done) (groups[e.session] ||= []).push(e);
    const head = `<div class="dp-head"><span>${done.length} 条录制${st.free != null ? ` · 剩余 ${st.free} GB` : ''}</span>
      <button class="txt" data-go="trash">回收站</button></div>`;
    const sel = st.sel.size ? `<div class="selbar"><span>已选 ${st.sel.size} 条</span>
      <button class="btn main sm" data-go="export">导出为数据集</button><button class="txt" data-act="clearsel">取消</button></div>` : '';
    if (!done.length) return head + `<p class="empty">还没有录制。在左侧“录制”开始第一条。</p>`;
    return head + sel + Object.entries(groups).map(([s, eps]) => `<section class="dp-group">
      <div class="dp-label"><span>${esc(sessionLabel(s))}</span><button class="txt sm" data-all="${esc(s)}">全选</button></div>
      ${eps.map((e) => `<div class="row ${e.discarded ? 'dim' : ''}" data-ep="${esc(e.id)}">
        <button class="check ${st.sel.has(e.id) ? 'on' : ''}" data-check="${esc(e.id)}" aria-label="选择"></button>
        <div class="row-main"><div class="row-title">${epTitle(e)}${tags(e)}</div>
          <div class="row-meta">${fmtClock(e.t_start).slice(0, 5)}${e.style ? ` · 风格 ${esc(e.style)}` : ''} · ${e.cams.length} 路视频 · ${e.mb} MB</div>
          ${e.note ? `<div class="row-note">${esc(e.note)}</div>` : ''}</div>
        <div class="row-side">${mmss(e.duration)}</div>
      </div>`).join('')}</section>`).join('');
  }

  function pageEpisode(e) {
    if (!e) return back('录制') + `<p class="empty">这条录制已不存在。</p>`;
    return `${back('录制')}
      <div class="detail-head"><div class="detail-title">${epTitle(e)}${tags(e)}</div>
        <div class="row-meta">${fmtDate(e.t_start)} ${fmtClock(e.t_start)} – ${fmtClock(e.t_start + e.duration)} · ${e.duration.toFixed(1)} 秒</div>
        <div class="row-meta">${esc(sessionLabel(e.session))} · ${e.cams.length} 路视频 · ${e.mb} MB</div></div>
      <div id="pb-videos"></div>
      <div class="dp-label"><span>事后补标</span><span class="hint">←/→ 逐帧 · Shift 1 秒 · 点画面放大</span></div>
      <div id="post-marks"><p class="hint">加载中…</p></div>
      <div class="dp-label"><span>事件</span><span class="hint">点击跳转</span></div>
      <div id="ev-list"></div>
      <div class="dp-label"><span>标注</span>${e.edits ? `<span class="hint">已修改 ${e.edits} 次</span>` : ''}</div>
      <form class="form" data-form="episode">
        <label class="field"><span>参与者</span><input name="participant" value="${esc(e.participant)}"></label>
        <label class="field"><span>场景</span><input name="scenario" value="${esc(e.scenario)}"></label>
        <label class="field"><span>风格</span><input name="style" value="${esc(e.style)}" placeholder="可空"></label>
        <label class="field"><span>版本</span><input name="version" value="${esc(e.version)}" inputmode="numeric"></label>
        <div class="field"><span>标记</span><div class="toggles">
          <button type="button" class="toggle ${e.final ? 'on' : ''}" data-flag="final">最终版本</button>
          <button type="button" class="toggle ${e.repeat_of_final ? 'on' : ''}" data-flag="repeat_of_final">重复</button>
          <button type="button" class="toggle danger ${e.discarded ? 'on' : ''}" data-flag="discarded">作废</button></div></div>
        <label class="field"><span>理由</span><input name="reason" value="${esc(e.reason)}" placeholder="修改 / 重录理由"></label>
        <label class="field tall"><span>备注</span><textarea name="note" rows="3">${esc(e.note)}</textarea></label>
        <div class="form-actions"><button type="submit" class="btn main">保存</button><span class="msg">${esc(st.msg)}</span>
          <button type="button" class="txt danger" data-act="trash" data-id="${esc(e.id)}">移到回收站</button></div>
      </form>`;
  }

  // ---------------------------------------------------------------- post-hoc marks (2026-11-11)
  // part-a-session-protocol: "receiver picks up the object" (接收方拿起物体) is marked afterwards on the third-person video. Marks live in the
  // episode's annotations.json (server keeps a log of every change); events.jsonl stays as recorded.
  const POST_KINDS = [{ kind: 'receiver_pickup', label: '接收方拿起物体', near: 'gripper_open', nearLabel: '跳到夹爪张开',
    hint: '在第三人称画面上停到接收方拿起物体的那一帧，再点“标记当前帧”' }];
  const SRC = { auto: '自动', researcher: '手动', post: '补标' };
  const fmt1 = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${(s % 60).toFixed(1).padStart(4, '0')}`;
  playback.onSeek = (t) => host.querySelectorAll('.mk-cur').forEach((x) => { x.textContent = fmt1(t); });
  function renderMarks() {
    const pm = $('#post-marks', host), el = $('#ev-list', host);
    if (!pm || !playback.active || playback.ep?.id !== st.page?.id) return;
    const t0 = playback.t0, marks = playback.marks();
    pm.innerHTML = POST_KINDS.map((k) => {
      const m = (playback.annots || []).find((a) => a.kind === k.kind);
      const near = playback.events.find((e) => e.kind === k.near);
      return `<div class="pm-item">
        <div class="row-title">${k.label}${m ? '<i class="tag accent">已标注</i>' : '<i class="tag">未标注</i>'}</div>
        <div class="row-meta">${m ? `${fmt1(m.t - t0)} · ${fmtClock(m.t, true)}` : k.hint}</div>
        <div class="pm-actions">
          ${near ? `<button class="txt" data-seek="${near.t - t0}">${k.nearLabel}</button>` : ''}
          ${m ? `<button class="txt" data-seek="${m.t - t0}">跳到标记</button>` : ''}
          <button class="btn sm main" data-mk-set="${k.kind}">${m ? '改为当前帧' : '标记当前帧'} <span class="mk-cur">${fmt1(playback.t)}</span></button>
          ${m ? `<button class="txt danger" data-mk-del="${k.kind}">删除</button>` : ''}
        </div></div>`;
    }).join('');
    el.innerHTML = marks.length ? marks.map((e) => `<button class="evrow" data-seek="${e.t - t0}"><b>${fmt1(e.t - t0)}</b>
        <span>${esc(e.label || e.kind)}</span><i class="tag ${e.source === 'post' ? 'accent' : ''}">${SRC[e.source] || e.source}</i></button>`).join('')
      : '<p class="empty">这条录制没有事件。</p>';
  }
  async function annotate(action, kind) {
    const k = POST_KINDS.find((x) => x.kind === kind);
    if (action === 'delete' && !confirm(`删除“${k.label}”的补标？修改记录会保留。`)) return;
    try {
      const r = await post('/api/episode/annotate', { id: st.page.id, action, kind, label: k.label, t: playback.t0 + playback.t });
      playback.setAnnotations(r.events); renderMarks();
    } catch (err) { alert('保存失败：' + err.message); }
  }

  function pageTrash() {
    return `${back('录制')}<div class="detail-head"><div class="detail-title">回收站</div>
      <div class="row-meta">移到这里的录制可以恢复，不会自动清除。</div></div>` +
      (st.trash.length ? st.trash.map((e) => `<div class="row static">
        <div class="row-main"><div class="row-title">${epTitle(e)}</div>
          <div class="row-meta">${fmtDate(e.t_start).slice(5)} ${fmtClock(e.t_start).slice(0, 5)} · ${e.mb} MB</div></div>
        <div class="row-side"><button class="btn sm" data-act="restore" data-id="${esc(e.id)}">恢复</button></div></div>`).join('')
        : `<p class="empty">回收站是空的。</p>`);
  }

  function pageExport() {
    const eps = st.episodes.filter((e) => st.sel.has(e.id));
    const scen = [...new Set(eps.map((x) => x.scenario))].join('-').replace(/[^A-Za-z0-9_-]/g, '') || 'data';
    const today = new Date().toISOString().slice(0, 10).replace(/-/g, '');
    return `${back('录制')}<div class="detail-head"><div class="detail-title">导出为训练数据集</div>
      <div class="row-meta">${eps.length} 条录制 · 存到机载端 ~/fmp_datasets/</div></div>
      <form class="form" data-form="export">
        <label class="field"><span>名称</span><input name="name" value="${esc(scen + '_' + today)}"></label>
        <div class="field"><span>手臂</span><div class="seg" data-seg="arms">${Object.entries(ARMS).map(([k, l], i) => `<button type="button" data-v="${k}" class="${i ? '' : 'on'}">${l}</button>`).join('')}</div></div>
        <div class="field"><span>相机</span><div class="toggles" data-multi="cams">${Object.entries(CAM).map(([k, l]) => `<button type="button" class="toggle on" data-v="${k}">${l}</button>`).join('')}</div></div>
        <div class="form-actions"><button type="submit" class="btn main">开始导出</button><span class="msg">${esc(st.msg)}</span></div>
      </form>
      <div class="dp-label"><span>包含</span></div>
      ${eps.map((e) => `<div class="row static"><div class="row-main"><div class="row-title">${epTitle(e)}${tags(e)}</div>
        <div class="row-meta">${esc(sessionLabel(e.session))} · ${fmtClock(e.t_start).slice(0, 5)}</div></div><div class="row-side">${mmss(e.duration)}</div></div>`).join('')}`;
  }

  function listDatasets() {
    if (!st.datasets.length) return `<p class="empty">还没有数据集。在“录制”里勾选几条，点“导出为数据集”。</p>`;
    return `<div class="dp-head"><span>${st.datasets.length} 个数据集 · ~/fmp_datasets</span></div>` + st.datasets.map((d) => `
      <div class="row" data-ds="${esc(d.name)}"><div class="row-main"><div class="row-title">${esc(d.name)}</div>
        <div class="row-meta">${d.episodes} 条 · ${ARMS[d.arms] || '—'} · ${(d.cams || []).map((c) => CAM[c] || c).join('、')}</div></div>
        <div class="row-side">${d.frames} 帧</div></div>`).join('');
  }

  function pageDataset(d) {
    if (!d) return back('数据集') + `<p class="empty">这个数据集已不存在。</p>`;
    const runs = st.jobs.filter((j) => j.kind === 'train' && j.dataset === d.name);
    const stamp = new Date().toISOString().slice(5, 16).replace(/[-T:]/g, '');
    const nCam = d.cams.length;
    return `${back('数据集')}<div class="detail-head"><div class="detail-title">${esc(d.name)}</div>
        <div class="row-meta">${d.episodes} 条 · ${d.frames} 帧 · ${d.fps} fps · ${ARMS[d.arms] || '—'}</div>
        <div class="row-meta">${d.created ? '导出于 ' + esc(d.created) : ''}</div></div>
      <div class="dp-label"><span>训练</span></div>
      <form class="form" data-form="train" data-ds="${esc(d.name)}">
        <label class="field"><span>名称</span><input name="name" value="${esc(d.name)}_act_${stamp}"></label>
        <div class="field"><span>步数</span><div class="seg" data-seg="steps"><button type="button" data-v="2000">试跑 2千</button><button type="button" data-v="20000" class="on">2万</button><button type="button" data-v="100000">10万</button></div></div>
        <div class="field"><span>相机</span><div class="toggles" data-multi="cams">${d.cams.map((c) => `<button type="button" class="toggle on" data-v="${c}">${CAM[c] || c}</button>`).join('')}</div></div>
        <div class="field"><span>批大小</span><div class="seg" data-seg="batch">${[4, 8, 16].map((b) => `<button type="button" data-v="${b}" class="${b === (nCam >= 3 ? 4 : 8) ? 'on' : ''}">${b}</button>`).join('')}</div></div>
        <p class="hint">显存 6 GB：三路相机用 4，一到两路可用 8。2万步约 2 小时，10万步约一晚；结束后自动离线检验。</p>
        <div class="form-actions"><button type="submit" class="btn main">开始训练</button><span class="msg">${esc(st.msg)}</span></div>
      </form>
      ${runs.length ? `<div class="dp-label"><span>这个数据集的训练</span></div>${runs.map(jobRow).join('')}` : ''}
      <div class="dp-label"><span>来源（${d.sources.length}）</span></div>
      ${d.sources.map((s) => `<div class="row static"><div class="row-main"><div class="row-meta mono">${esc(s)}</div></div></div>`).join('')}`;
  }

  function jobRow(j) {
    const p = j.progress;
    const pct = p && p.total ? Math.min(100, Math.round((p.done / p.total) * 100)) : 0;
    const [label, kind] = JOB[j.state] || [j.state, 'muted'];
    return `<div class="row static job"><div class="row-main">
        <div class="row-title">${esc(j.title)}<i class="tag ${kind}">${label}</i></div>
        ${j.state === 'running' ? `<div class="progress"><i style="width:${pct}%"></i></div>` : ''}
        <div class="row-meta">${new Date(j.start * 1000).toLocaleString('zh-CN', { hour12: false, month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' })}${p && ['running', 'done'].includes(j.state) ? ` · ${p.done}/${p.total}` : ''}</div>
        ${j.state === 'failed' && j.tail?.length ? `<div class="row-note danger">${esc(j.tail[0]).slice(0, 140)}</div>` : ''}
        <div class="row-links">${j.kind === 'train' ? `<a class="txt sm" href="http://${location.hostname}:8090/?run=${encodeURIComponent(j.run || '')}">训练曲线</a>` : ''}
          ${j.state === 'running' ? `<button class="txt sm danger" data-act="cancel" data-id="${esc(j.id)}">取消</button>` : ''}</div>
      </div></div>`;
  }
  function listJobs() {
    return st.jobs.length ? `<div class="dp-head"><span>导出与训练</span></div>` + st.jobs.map(jobRow).join('')
      : `<p class="empty">还没有任务。</p>`;
  }

  // ---------------------------------------------------------------- interaction
  host.addEventListener('click', async (ev) => {
    const t = ev.target.closest('button, [data-ep], [data-ds], a');
    if (!t || !host.contains(t)) return;
    if (t.tagName === 'A') return;
    if (t.dataset.tab) return tab(t.dataset.tab);
    if (t.dataset.go === 'back') return go(null);
    if (t.dataset.go === 'trash') { await load(['trash']); return go({ type: 'trash' }); }
    if (t.dataset.go === 'export') return go({ type: 'export' });
    if (t.dataset.check) { ev.stopPropagation(); st.sel.has(t.dataset.check) ? st.sel.delete(t.dataset.check) : st.sel.add(t.dataset.check); return render(); }
    if (t.dataset.all) {
      const ids = st.episodes.filter((e) => e.session === t.dataset.all && !e.discarded && e.duration != null).map((e) => e.id);
      const every = ids.every((id) => st.sel.has(id));
      ids.forEach((id) => (every ? st.sel.delete(id) : st.sel.add(id)));
      return render();
    }
    if (t.dataset.act === 'clearsel') { st.sel.clear(); return render(); }
    if (t.dataset.seek != null) { playback.pause(); return playback.seek(+t.dataset.seek); }
    if (t.dataset.mkSet) return annotate('set', t.dataset.mkSet);
    if (t.dataset.mkDel) return annotate('delete', t.dataset.mkDel);
    if (t.dataset.flag) return t.classList.toggle('on');
    if (t.dataset.v && t.parentElement.dataset.seg) { t.parentElement.querySelectorAll('button').forEach((b) => b.classList.toggle('on', b === t)); return; }
    if (t.dataset.v && t.parentElement.dataset.multi) {
      t.classList.toggle('on');
      const f = t.closest('form');
      if (f?.dataset.form === 'train') {  // 2026-11-10: 3 cameras x batch 8 ran out of the 6 GB GPU memory
        const n = f.querySelectorAll('[data-multi="cams"] .on').length;
        f.querySelectorAll('[data-seg="batch"] button').forEach((b) => b.classList.toggle('on', b.dataset.v === (n >= 3 ? '4' : '8')));
      }
      return;
    }
    if (t.dataset.act === 'trash') {
      const e = st.episodes.find((x) => x.id === t.dataset.id);
      if (!confirm(`把 #${e.episode} ${e.scenario} v${e.version} 移到回收站？之后可以在回收站恢复。`)) return;
      try { await post('/api/episode/trash', { id: e.id }); st.sel.delete(e.id); await load(['episodes']); go(null); }
      catch (err) { st.msg = '失败：' + err.message; render(); }
      return;
    }
    if (t.dataset.act === 'restore') {
      try { await post('/api/trash/restore', { id: t.dataset.id }); await load(['episodes', 'trash']); render(); }
      catch (err) { alert('恢复失败：' + err.message); }
      return;
    }
    if (t.dataset.act === 'cancel') {
      if (!confirm('取消这个任务？导出到一半的数据集会被删除；训练停在当前进度。')) return;
      await post('/api/jobs/cancel', { id: t.dataset.id }).catch(() => {});
      await load(['jobs']); return render();
    }
    if (ev.target.closest('[data-ep]') && !t.dataset.check) return go({ type: 'episode', id: t.closest('[data-ep]').dataset.ep });
    if (ev.target.closest('[data-ds]') && t.dataset.ds && !t.closest('form')) return go({ type: 'dataset', name: t.dataset.ds });
  });

  host.addEventListener('submit', async (ev) => {
    ev.preventDefault();
    const f = ev.target, kind = f.dataset.form;
    const val = (n) => f.elements[n]?.value.trim();
    const seg = (n) => f.querySelector(`[data-seg="${n}"] .on`)?.dataset.v;
    const multi = (n) => [...f.querySelectorAll(`[data-multi="${n}"] .on`)].map((b) => b.dataset.v);
    try {
      if (kind === 'episode') {
        const changes = { participant: val('participant'), scenario: val('scenario'), style: val('style'),
          version: parseInt(val('version'), 10) || undefined, reason: f.elements.reason.value, note: f.elements.note.value };
        f.querySelectorAll('[data-flag]').forEach((b) => { changes[b.dataset.flag] = b.classList.contains('on'); });
        const r = await post('/api/episode/update', { id: st.page.id, changes });
        await load(['episodes']);
        st.msg = r.changed.length ? `已保存 ${r.changed.length} 项` : '没有改动';
        $('.msg', f).textContent = st.msg;
        const frag = document.createRange().createContextualFragment(pageEpisode(st.episodes.find((x) => x.id === st.page.id)));
        $('.detail-head', host).replaceWith($('.detail-head', frag));
      } else if (kind === 'export') {
        await post('/api/export', { name: val('name'), episodes: [...st.sel], arms: seg('arms'), cams: multi('cams') });
        st.sel.clear(); await load(['jobs']); st.tab = 'jobs'; go(null);
      } else if (kind === 'train') {
        await post('/api/train', { dataset: f.dataset.ds, name: val('name'), steps: +seg('steps'), batch: +seg('batch'), cams: multi('cams') });
        await load(['jobs']); st.tab = 'jobs'; go(null);
      }
      poll();
    } catch (err) {
      st.msg = err.message; const m = $('.msg', f); if (m) m.textContent = err.message;
    }
  });

  return {
    playback,
    async show(on) {
      visible = on;
      if (!on) { clearTimeout(timer); if (st.page?.type === 'episode') go(null); return; }
      await load(); render(); poll();
    },
    async episodesChanged() { if (visible) { await load(['episodes']); if (!isTyping() && st.page?.type !== 'episode') render(); } },
  };
}
