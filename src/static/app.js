"use strict";

const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");
let state = null;
let view = { scale: 26, ox: 430, oy: 560, pitch: 0.75, yaw: -0.5 };
let projected = [];

const palette = [
  "#4cc2ff", "#3fd07f", "#f2b21b", "#c084fc", "#ff8f70",
  "#7ee787", "#ffa657", "#79c0ff", "#d2a8ff", "#56d4dd",
];

function hsl(h, s, l) {
  return `hsl(${h},${s}%,${l}%)`;
}

async function loadState() {
  const r = await fetch("/api/state");
  state = await r.json();
  fitView();
  render();
  renderPanel();
}

function bounds() {
  const pts = state.points.filter((p) => p.aligned).map((p) => [p.x, p.y, p.z || 0]);
  state.poses.forEach((p) => pts.push([p.x, p.y, p.z || 0]));
  if (!pts.length) return { min: [-1, -1, -1], max: [1, 1, 1] };
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (const p of pts)
    for (let i = 0; i < 3; i++) {
      min[i] = Math.min(min[i], p[i]);
      max[i] = Math.max(max[i], p[i]);
    }
  for (let i = 0; i < 3; i++)
    if (max[i] - min[i] < 0.5) {
      max[i] += 0.25;
      min[i] -= 0.25;
    }
  return { min, max };
}

function fitView() {
  const b = bounds();
  const w = Math.max(b.max[0] - b.min[0], b.max[1] - b.min[1]);
  view.scale = Math.min(760, 520) / Math.max(w, 1) * 0.9;
  const cx = (b.min[0] + b.max[0]) / 2;
  const cy = (b.min[1] + b.max[1]) / 2;
  view.ox = 430 - cx * view.scale;
  view.oy = 560 + cy * view.scale;
}

function project(p) {
  const mode = document.getElementById("view").value;
  let x, y;
  if (mode === "xy") {
    x = view.ox + p[0] * view.scale;
    y = view.oy - p[1] * view.scale;
  } else if (mode === "xz") {
    x = view.ox + p[0] * view.scale;
    y = view.oy - (p[2] || 0) * view.scale;
  } else {
    const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
    const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
    const rx = p[0] * cy - p[1] * sy;
    const ry0 = p[0] * sy + p[1] * cy;
    const ry = ry0 * cp - (p[2] || 0) * sp;
    const rz = ry0 * sp + (p[2] || 0) * cp;
    const persp = 1 / (1 + rz * 0.05);
    x = view.ox + rx * view.scale * persp;
    y = view.oy - ry * view.scale * persp;
  }
  return [x, y];
}

function pointColor(p) {
  const mode = document.getElementById("colorMode").value;
  if (!p.aligned) return "#5b6670";
  if (mode === "frame") return palette[(p.color_group - 1) % palette.length];
  if (mode === "aligned") return "#3fd07f";
  const t0 = state.points[0]?.t ?? 0;
  const t1 = state.points[state.points.length - 1]?.t ?? t0 + 1;
  const u = Math.max(0, Math.min(1, (p.t - t0) / Math.max(t1 - t0, 1e-6)));
  return hsl(210 - u * 200, 80, 60);
}

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawGaps();
  drawTrajectory();
  drawPoints();
  drawLegend();
}

function drawGrid() {
  ctx.strokeStyle = "#17202a";
  ctx.lineWidth = 1;
  for (let gx = -20; gx <= 20; gx++) {
    const [x1] = project([gx, -20, 0]);
    const [x2] = project([gx, 20, 0]);
    ctx.beginPath(); ctx.moveTo(x1, 0); ctx.lineTo(x2, canvas.height); ctx.stroke();
  }
  for (let gy = -20; gy <= 20; gy++) {
    const [, y1] = project([-20, gy, 0]);
    const [, y2] = project([20, gy, 0]);
    ctx.beginPath(); ctx.moveTo(0, y1); ctx.lineTo(canvas.width, y2); ctx.stroke();
  }
}

function drawGaps() {
  ctx.fillStyle = "rgba(242,178,27,.08)";
  ctx.strokeStyle = "rgba(242,178,27,.55)";
  ctx.setLineDash([5, 4]);
  for (const g of state.gaps) {
    const before = state.poses.find((p) => Math.abs(p.t - g.from_t) < 1e-6 && p.generation_id === g.generation_id)
      || state.poses.filter((p) => p.t <= g.from_t).pop();
    const after = state.poses.find((p) => Math.abs(p.t - g.to_t) < 1e-6 && p.generation_id === g.generation_id)
      || state.poses.filter((p) => p.t >= g.to_t)[0];
    if (before && after) {
      const [x1, y1] = project([before.x, before.y, before.z]);
      const [x2, y2] = project([after.x, after.y, after.z]);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
    if (g.kind === "pose_gap" && before && after) {
      const [mx, my] = project([(before.x + after.x) / 2, (before.y + after.y) / 2, 0]);
      ctx.fillStyle = "#f2b21b";
      ctx.fillText(`缺口 ${g.seconds.toFixed(2)}s`, mx + 4, my - 4);
      ctx.fillStyle = "rgba(242,178,27,.08)";
    }
  }
  ctx.setLineDash([]);
}

function drawTrajectory() {
  const byGen = {};
  for (const p of state.poses)
    (byGen[p.generation_id] ||= []).push(p);
  for (const [g, ps] of Object.entries(byGen)) {
    ctx.strokeStyle = palette[(+g - 1) % palette.length];
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ps.forEach((p, i) => {
      const [x, y] = project([p.x, p.y, p.z || 0]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.globalAlpha = 1;
    for (const p of ps) {
      const [x, y] = project([p.x, p.y, p.z || 0]);
      ctx.fillStyle = "#0e1116";
      ctx.beginPath(); ctx.arc(x, y, 3.2, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = ctx.strokeStyle;
    }
  }
}

function drawPoints() {
  projected = [];
  for (const p of state.points) {
    const q = p.aligned ? [p.x, p.y, p.z || 0] : sensorStub(p);
    const [x, y] = project(q);
    projected.push({ id: p.id, x, y, aligned: p.aligned });
    ctx.fillStyle = pointColor(p);
    ctx.globalAlpha = p.aligned ? 0.9 : 0.4;
    ctx.beginPath();
    ctx.arc(x, y, p.aligned ? 2.2 : 1.8, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.globalAlpha = 1;
}

function sensorStub(p) {
  // Unaligned points have no map coords: park them on the x axis at time.
  return [p.t - (state.points[0]?.t ?? 0), -3.2, 0];
}

function drawLegend() {
  const gens = state.generations.map((g, i) => `${palette[i % palette.length]} ● 代次 ${g.id} (${g.device_id}, ${g.reason})`).join("\n");
  document.getElementById("legend").textContent =
    gens + "\n#f2b21b ┇ 时间缺口（姿态插值禁止跨越）\n#5b6670 ● 未对齐点（无合法路径/插值）";
}

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

function renderPanel() {
  const aligned = state.points.filter((p) => p.aligned).length;
  const blocksDone = state.blocks.filter((b) => b.status === "complete").length;
  document.getElementById("summary").innerHTML = `
    <div class="kv">
      <span class="k">采集代次</span><span>${state.generations.length}</span>
      <span class="k">派生块</span><span>${blocksDone} complete / ${state.blocks.length}</span>
      <span class="k">点</span><span>${aligned} 对齐 / ${state.points.length - aligned} 未对齐</span>
      <span class="k">变换边版本</span><span>${state.edges.length}</span>
      <span class="k">时间缺口</span><span>${state.gaps.length}</span>
    </div>
    <div style="margin-top:6px">
      ${state.generations
        .map((g) => `<div><span class="tag ok">代次 ${g.id}</span>${esc(g.device_id)} · ${esc(g.reason)} · ${g.packet_count} 包</div>`)
        .join("")}
    </div>`;

  const routes = document.getElementById("routes");
  if (!state.routes.length) {
    routes.innerHTML = "<div class='muted'>暂无已对齐路径。</div>";
  } else {
    routes.innerHTML = state.routes
      .map(
        (r) => `<div class="route">
          <div><b>${esc(r.sensor)}</b> @t=${r.t.toFixed(3)}s
          <span class="tag ok">选中</span> ${r.selected.map(esc).join(" → ")}
          <div class="mono">精度成本 trace(Σ)=${r.cost.toExponential(3)}
          （越小越精确；并列时版本高者胜，再按帧链字典序）</div></div>
          ${r.candidates
            .map(
              (c) =>
                `<div class="cand ${c.selected ? "selected" : "dropped"}">
                  <span class="tag ${c.selected ? "ok" : "warn"}">${c.selected ? "采用" : "候选保留"}</span>
                  ${c.frames.map(esc).join(" → ")}
                  <div class="mono">cost=${c.precision_cost.toExponential(3)} · vscore=${c.version_score}<br/>${esc(c.reason)}</div>
                </div>`
            )
            .join("")}
        </div>`
      )
      .join("");
  }
}

canvas.addEventListener("click", (e) => {
  const rect = canvas.getBoundingClientRect();
  const mx = (e.clientX - rect.left) * (canvas.width / rect.width);
  const my = (e.clientY - rect.top) * (canvas.height / rect.height);
  let hit = null;
  let best = 12;
  for (const q of projected) {
    const d = Math.hypot(q.x - mx, q.y - my);
    if (d < best) {
      best = d;
      hit = q;
    }
  }
  if (hit) showProvenance(hit.id);
});

async function showProvenance(id) {
  const el = document.getElementById("provenance");
  el.innerHTML = "<div class='muted'>查询来源链…</div>";
  const r = await fetch(`/api/point/${id}`);
  if (!r.ok) {
    el.innerHTML = "<span class='tag bad'>错误</span> 无法读取来源链";
    return;
  }
  const p = await r.json();
  const o = p.point;
  el.innerHTML = `
    <div class="kv">
      <span class="k">派生点</span><span>#${id}</span>
      <span class="k">原始包</span><span class="mono">#${o.packet_id} (seq ${o.raw_seq})</span>
      <span class="k">采集代次</span><span>${o.generation_id}</span>
      <span class="k">设备</span><span>${esc(o.device_id)}</span>
      <span class="k">原始时间戳</span><span class="mono">sow=${o.raw_timestamp_sow.toFixed(4)}
        week=${o.raw_week ?? "—"} scale=${esc(o.time_scale)}</span>
      <span class="k">坐标系</span><span>${esc(o.coord_frame)}</span>
      <span class="k">单位</span><span>${esc(o.units)}（原始保存，派生缓存用 SI）</span>
      <span class="k">内容摘要</span><span>${esc(o.content_summary)}</span>
      <span class="k">原始坐标</span><span class="mono">[${o.raw_sensor_xyz.map((v)=>v.toFixed(3)).join(", ")}]</span>
      <span class="k">SI 坐标</span><span class="mono">[${o.si_sensor_xyz.map((v)=>v.toFixed(3)).join(", ")}]</span>
      <span class="k">地图坐标</span><span class="mono">${
        p.final_map_xyz ? "[" + p.final_map_xyz.map((v) => v.toFixed(3)).join(", ") + "]" : "未对齐"
      }</span>
      <span class="k">位置 1σ</span><span class="mono">${
        p.final_sigma_position_m
          ? "[" + p.final_sigma_position_m.map((v) => v.toExponential(2)).join(", ") + "] m"
          : "—"
      }</span>
    </div>
    <h2 style="font-size:13px">经过的变换版本（误差逐跳传播）</h2>
    ${p.chain
      .map(
        (h) => `<div class="hop">
          <span class="tag ${h.arc_id === "UNRESOLVED" ? "bad" : "ok"}">
            ${h.arc_id === "UNRESOLVED" ? "中断" : "变换"}</span>
          ${esc(h.from)} → ${esc(h.to)}
          <div class="mono">${esc(h.arc_id)}<br/>版本: ${esc(h.version)}
          <br/>该跳 Σ trace=${h.covariance_trace != null ? h.covariance_trace.toExponential(3) : "—"}</div>
        </div>`
      )
      .join("")}
    <h2 style="font-size:13px">候选路径（未选路径保留可核验）</h2>
    ${p.candidates
      .map(
        (c) => `<div class="cand ${c.selected ? "selected" : "dropped"}">
          <span class="tag ${c.selected ? "ok" : "warn"}">${c.selected ? "选中" : "候选"}</span>
          ${c.frames.map(esc).join(" → ")}
          <div class="mono">cost=${c.precision_cost.toExponential(3)} vscore=${c.version_score}<br/>${esc(c.reason)}</div>
        </div>`
      )
      .join("")}`;
}

let dragging = null;
canvas.addEventListener("mousedown", (e) => {
  dragging = { x: e.clientX, y: e.clientY, ox: view.ox, oy: view.oy };
});
window.addEventListener("mouseup", () => (dragging = null));
window.addEventListener("mousemove", (e) => {
  if (!dragging) return;
  view.ox = dragging.ox + (e.clientX - dragging.x);
  view.oy = dragging.oy + (e.clientY - dragging.y);
  render();
});
canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const f = e.deltaY < 0 ? 1.12 : 0.89;
  view.scale *= f;
  render();
}, { passive: false });

document.getElementById("colorMode").addEventListener("change", render);
document.getElementById("view").addEventListener("change", render);
document.getElementById("refresh").addEventListener("click", loadState);
document.getElementById("reset").addEventListener("click", async () => {
  const r = await fetch("/api/demo/reset", { method: "POST" });
  state = await r.json();
  fitView();
  render();
  renderPanel();
});
document.getElementById("revise").addEventListener("click", async () => {
  // A new calibration version for gen-A window only: only blocks intersecting
  // that window are invalidated and rebuilt.
  const body = {
    source: "lidar/survey-1",
    target: "body/survey-1",
    valid_from: 1000.0,
    valid_to: 1010.5,
    translation: [0.125, 0.002, -0.048],
    quat: [1, 0, 0, 0],
    covariance: [
      6e-7, 6e-7, 6e-7,
      6e-9, 6e-9, 6e-9,
    ],
    origin: "calibration-v3",
  };
  const r = await fetch("/api/revise", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!r.ok) {
    const j = await r.json().catch(() => ({}));
    alert(`标定被拒绝: ${j.message || r.status}`);
  }
  await loadState();
});

loadState();
