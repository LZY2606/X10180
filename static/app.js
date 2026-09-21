const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
let view = 'xy';
let data = { frames: [], poses: [], packets: [], edges: [], blocks: [], state: {} };
let projected = [];

const COLORS = ['#5fd68c', '#4ea1ff', '#ffb84d', '#c39bff', '#ff8f6b',
  '#6be3e3', '#f06bd6', '#9bd06b', '#e3e36b', '#6b8fe3'];

async function getJson(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(url + ' -> ' + r.status);
  return r.json();
}

async function refresh() {
  const [points, traj, packets, edges, blocks, state] = await Promise.all([
    getJson('/api/points'), getJson('/api/trajectory'), getJson('/api/packets'),
    getJson('/api/edges'), getJson('/api/blocks'), getJson('/api/state'),
  ]);
  data = { ...points, poses: traj.poses, packets, edges, blocks, state };
  document.getElementById('state').textContent =
    `${state.packets} 包 · ${state.frames} 帧 · ${state.edges} 变换 · ` +
    `${state.blocks_done} 块完成 / ${state.blocks_blocked} 阻塞 / ${state.map_points} 点`;
  renderPanels();
  draw();
  await renderPaths();
}

function xyOf(p) {
  return view === 'xy' ? [p[0], -p[1]] : [p[0], -p[2]];
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  canvas.width = w * dpr; canvas.height = h * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const all = [];
  data.frames.forEach(f => f.points.forEach(p => all.push(p.map)));
  data.poses.forEach(s => all.push(s.t));
  if (!all.length) return;
  const min = [Infinity, Infinity], max = [-Infinity, -Infinity];
  all.forEach(p => {
    const [x, y] = xyOf(p);
    min[0] = Math.min(min[0], x); min[1] = Math.min(min[1], y);
    max[0] = Math.max(max[0], x); max[1] = Math.max(max[1], y);
  });
  const pad = 50;
  const sx = (w - pad * 2) / Math.max(max[0] - min[0], 1e-6);
  const sy = (h - pad * 2) / Math.max(max[1] - min[1], 1e-6);
  const s = Math.min(sx, sy);
  const project = p => {
    const [x, y] = xyOf(p);
    return [pad + (x - min[0]) * s, pad + (y - min[1]) * s];
  };

  // Grid.
  ctx.strokeStyle = '#1b2431'; ctx.lineWidth = 1;
  for (let i = 0; i <= 8; i++) {
    const gx = pad + (w - pad * 2) * i / 8;
    ctx.beginPath(); ctx.moveTo(gx, pad); ctx.lineTo(gx, h - pad); ctx.stroke();
  }

  // Trajectories colored by device; generation breaks drawn as gaps.
  const devices = {};
  data.poses.forEach(s => (devices[s.device] ||= []).push(s));
  Object.entries(devices).forEach(([dev, list]) => {
    const primary = dev.includes('primary');
    ctx.strokeStyle = primary ? '#4ea1ff' : '#ffb84d';
    ctx.fillStyle = ctx.strokeStyle; ctx.lineWidth = 1.5;
    let prev = null;
    list.forEach(s => {
      const q = project(s.t);
      if (prev && s.generation === prev.gen) {
        const gap = s.time_ms - prev.time;
        ctx.setLineDash(gap > 2000 ? [6, 5] : []);
        if (gap > 2000) ctx.strokeStyle = '#ff6b6b';
        ctx.beginPath(); ctx.moveTo(prev.q[0], prev.q[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
        ctx.strokeStyle = primary ? '#4ea1ff' : '#ffb84d';
        ctx.setLineDash([]);
      }
      ctx.beginPath(); ctx.arc(q[0], q[1], 3, 0, Math.PI * 2); ctx.fill();
      prev = { q, gen: s.generation, time: s.time_ms };
    });
  });

  // Points colored by frame index; record hit targets in map order.
  projected = [];
  data.frames.forEach((f, fi) => {
    const color = COLORS[fi % COLORS.length];
    ctx.fillStyle = color;
    f.points.forEach(p => {
      const q = project(p.map);
      ctx.beginPath(); ctx.arc(q[0], q[1], 2.4, 0, Math.PI * 2); ctx.fill();
      projected.push({ id: p.id, x: q[0], y: q[1], frame: fi, time: f.time_ms });
    });
  });

  // Time-gap markers between frames in the same/different generation.
  ctx.fillStyle = '#ff6b6b'; ctx.font = '11px sans-serif';
  for (let i = 1; i < data.frames.length; i++) {
    const a = data.frames[i - 1], b = data.frames[i];
    const gap = b.time_ms - a.time_ms;
    if (gap > 2000 || b.generation !== a.generation) {
      const pa = project(a.points.length ? a.points[Math.floor(a.points.length / 2)].map : [0, 0, 0]);
      const pb = project(b.points.length ? b.points[Math.floor(b.points.length / 2)].map : [0, 0, 0]);
      ctx.fillText(`缺口 ${gap}ms · 代次 ${a.generation}→${b.generation}`,
        (pa[0] + pb[0]) / 2 - 40, (pa[0] + pb[0]) ? (pa[1] + pb[1]) / 2 - 8 : 30);
    }
  }
}

canvas.addEventListener('click', async (e) => {
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left, my = e.clientY - rect.top;
  let best = null, bd = 9;
  for (const p of projected) {
    const d = Math.hypot(p.x - mx, p.y - my);
    if (d < bd) { bd = d; best = p; }
  }
  if (!best) return;
  const detail = await getJson('/api/point/' + best.id);
  const body = document.getElementById('point-body');
  const cov = detail.point_covariance;
  const chainRows = detail.chain.map((h, i) =>
    `<tr><td>${i + 1}</td><td>${h.edge} v${h.version}</td>
     <td>${h.from_frame}→${h.to_frame}${h.reversed ? ' (逆向)' : ''}${h.interpolated ? ' (插值)' : ''}</td>
     <td>${h.covariance_trace_after.toExponential(2)}</td></tr>`).join('');
  body.innerHTML = `
    <span class="k">点ID</span><span class="mono">#${detail.map_point_id}（帧 ${detail.frame_id}，块 ${detail.block_id}）</span>
    <span class="k">原始坐标</span><span class="mono">[${detail.raw.map(v => v.toFixed(3))}] m</span>
    <span class="k">地图坐标</span><span class="mono">[${detail.map.map(v => v.toFixed(3))}] m</span>
    <span class="k">点协方差</span><span class="mono">${cov.map(r => '[' + r.map(v => v.toExponential(1)).join(',') + ']').join(' ')}</span>
    <span class="k">总变换协方差迹</span><span class="mono">${trace6(detail.total_covariance).toExponential(3)}</span>
    <span class="k">变换版本</span><span class="mono">[${detail.edge_versions.join(', ')}]</span>
    <span class="k">误差传播链</span><span>
      <table><tr><th>#</th><th>变换</th><th>帧</th><th>累计迹</th></tr>${chainRows}</table>
    </span>`;
});

function trace6(v) {
  let t = 0;
  for (let i = 0; i < 6; i++) t += v[i * 6 + i];
  return t;
}

function tag(cls, text) {
  return `<span class="tag ${cls}">${text}</span>`;
}

function renderPanels() {
  document.getElementById('blocks').innerHTML =
    `<table>${data.blocks.map(b =>
      `<tr><td>#${b.block_id}</td><td>帧${b.frame_id}</td><td>${tag(b.status, b.status)}</td>
       <td class="mono">${b.edge_versions || ''}</td><td>${b.error ? b.error : ''}</td></tr>`
    ).join('')}</table>`;
  document.getElementById('packets').innerHTML =
    `<table>${data.packets.map(p =>
      `<tr><td>#${p.id}</td><td>${p.device_id}</td><td>代${p.generation}</td><td>序${p.seq}</td>
       <td>${tag(p.flag, p.flag)}</td></tr>
       <tr><td></td><td colspan="4" class="mono" style="color:#8b9bb0">
       ${p.raw_clock} ${p.raw_ts} · ${p.coord_frame} · ${p.unit} · ${p.content_summary}</td></tr>`
    ).join('')}</table>`;
  document.getElementById('edges').innerHTML =
    data.edges.map(e =>
      `<div>v${e.version} ${e.name}: ${e.from_frame}→${e.to_frame}
       ${e.dynamic ? '(动态)' : ''} t=[${e.transform.t[0].toFixed(2)},${e.transform.t[1].toFixed(2)},${e.transform.t[2].toFixed(2)}]
       迹=${traceOf(e.covariance).toExponential(1)}</div>`).join('');
}

function traceOf(flat) {
  let t = 0;
  for (let i = 0; i < 6; i++) t += flat[i * 6 + i];
  return t;
}

async function renderPaths() {
  const f = data.frames.find(x => x.points.length);
  if (!f) { document.getElementById('paths').textContent = '无帧'; return; }
  try {
    const paths = await getJson(`/api/path?from=lidar&to=map&time=${f.time_ms}`);
    document.getElementById('paths').innerHTML = paths.map((p, i) =>
      `<div class="candidate ${p.selected ? 'selected' : ''}">
        ${p.selected ? '★ 选用' : '　候选'} #${i + 1} 迹=${p.precision_trace.toExponential(2)}
        版本=[${p.version_key.join(',')}]<br>
        ${p.frames.join(' → ')}<br><span style="color:#8b9bb0">${p.rank_reason}</span></div>`
    ).join('<hr style="border-color:#222">');
  } catch (e) {
    document.getElementById('paths').textContent = '当前帧无可用路径（可能被阻塞）';
  }
}

document.querySelectorAll('.bar button').forEach(b =>
  b.addEventListener('click', () => { view = b.dataset.view; draw(); }));
document.getElementById('btn-recompute').onclick = refresh;
document.getElementById('btn-reset').onclick = async () => {
  await fetch('/api/demo/reset', { method: 'POST' });
  refresh();
};
window.addEventListener('resize', draw);
refresh();
