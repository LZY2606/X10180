//! Axum HTTP 服务：REST API + 随服务提供的静态前端（不下载任何外部资源）。

use crate::derive;
use crate::graph::{Edge, Graph};
use crate::math::{Mat6, Quat, SE3, Vec3};
use crate::store::Store;
use axum::{
    extract::{Query, State},
    http::StatusCode,
    response::{Html, Json},
    routing::{get, post},
    Router,
};
use serde::Deserialize;
use std::sync::{Arc, Mutex};

pub type Shared = Arc<Mutex<Store>>;

pub fn app(store: Shared) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/packets", post(ingest_packet).get(list_packets))
        .route("/api/transforms", post(add_transform).get(list_transforms))
        .route("/api/path", get(find_path))
        .route("/api/derive", post(run_derive))
        .route("/api/cloud", get(cloud))
        .route("/api/poses", get(poses))
        .route("/api/provenance", get(provenance))
        .route("/api/invalidate", get(invalidate_info))
        .with_state(store)
}

async fn index() -> Html<&'static str> {
    Html(include_str!("../static/index.html"))
}

fn err(e: impl std::fmt::Display) -> (StatusCode, Json<serde_json::Value>) {
    (StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": e.to_string()})))
}

#[derive(Deserialize)]
struct PacketIn {
    device: String,
    kind: String,
    seq: u64,
    t_raw: f64,
    frame: String,
    unit: String,
    summary: String,
    payload: serde_json::Value,
}

async fn ingest_packet(
    State(s): State<Shared>,
    Json(p): Json<PacketIn>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let mut st = s.lock().unwrap();
    let pkt = st
        .ingest(&p.device, &p.kind, p.seq, p.t_raw, &p.frame, &p.unit, &p.summary, p.payload)
        .map_err(err)?;
    Ok(Json(serde_json::json!({"id": pkt.id, "epoch": pkt.epoch, "t": pkt.t})))
}

async fn list_packets(State(s): State<Shared>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    let pkts = st.packets(None).unwrap_or_default();
    Json(serde_json::json!(pkts.iter().map(|p| serde_json::json!({
        "id": p.id, "device": p.device, "kind": p.kind, "seq": p.seq,
        "t_raw": p.t_raw, "frame": p.frame, "unit": p.unit,
        "summary": p.summary, "epoch": p.epoch, "t": p.t,
    })).collect::<Vec<_>>()))
}

#[derive(Deserialize)]
struct TransformIn {
    src: String,
    dst: String,
    rot: Quat,
    trans: Vec3,
    cov: Mat6,
    valid_from: f64,
    valid_to: f64,
}

async fn add_transform(
    State(s): State<Shared>,
    Json(t): Json<TransformIn>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let mut st = s.lock().unwrap();
    // 环路检测：拒绝时返回环路与累积残差
    let edges = st.transforms().map_err(err)?;
    let mut g = Graph::default();
    for e in edges {
        g.add_edge(e);
    }
    let new_edge = Edge {
        src: t.src.clone(), dst: t.dst.clone(), version: 0,
        tf: SE3::new(t.rot, t.trans), cov: t.cov,
        valid_from: t.valid_from, valid_to: t.valid_to,
    };
    if let Err(c) = g.check_cycle(&new_edge) {
        return Err(err(c));
    }
    let e = st
        .add_transform(&t.src, &t.dst, SE3::new(t.rot, t.trans), t.cov, t.valid_from, t.valid_to)
        .map_err(err)?;
    // 标定修订：只使覆盖时间段内的派生块失效
    let invalidated = st.invalidate_blocks(&t.src, &t.dst, t.valid_from, t.valid_to).map_err(err)?;
    Ok(Json(serde_json::json!({"version": e.version, "invalidated_blocks": invalidated})))
}

async fn list_transforms(State(s): State<Shared>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    let ts = st.transforms().unwrap_or_default();
    Json(serde_json::json!(ts.iter().map(|e| serde_json::json!({
        "src": e.src, "dst": e.dst, "version": e.version,
        "valid_from": e.valid_from, "valid_to": e.valid_to,
        "cov_trace": e.cov.trace(),
    })).collect::<Vec<_>>()))
}

#[derive(Deserialize)]
struct PathQ {
    from: String,
    to: String,
}

async fn find_path(State(s): State<Shared>, Query(q): Query<PathQ>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    let mut g = Graph::default();
    for e in st.transforms().unwrap_or_default() {
        g.add_edge(e);
    }
    let paths = g.find_paths(&q.from, &q.to);
    let ser = |p: &crate::graph::Path| serde_json::json!({
        "frames": p.frames,
        "versions": p.edges.iter().map(|e| e.version).collect::<Vec<_>>(),
        "cov_trace": p.cov.trace(),
    });
    Json(serde_json::json!({
        "chosen": paths.first().map(ser),
        "candidates": paths.iter().skip(1).map(ser).collect::<Vec<_>>(),
    }))
}

#[derive(Deserialize)]
struct DeriveIn {
    sensor_frame: String,
    body_frame: String,
    map_frame: String,
}

async fn run_derive(
    State(s): State<Shared>,
    Json(d): Json<DeriveIn>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let mut st = s.lock().unwrap();
    let stats = derive::compute_missing(&mut st, &d.sensor_frame, &d.body_frame, &d.map_frame).map_err(err)?;
    Ok(Json(serde_json::json!(stats)))
}

async fn cloud(State(s): State<Shared>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    let blocks = st.blocks().unwrap_or_default();
    let pkts = st.packets(None).unwrap_or_default();
    // 时间缺口：同设备同代次相邻包间隔超过阈值
    let mut gaps = Vec::new();
    let mut sorted: Vec<_> = pkts.iter().collect();
    sorted.sort_by(|a, b| {
        a.device.cmp(&b.device).then(a.epoch.cmp(&b.epoch)).then(a.t.partial_cmp(&b.t).unwrap())
    });
    for w in sorted.windows(2) {
        if w[0].device == w[1].device && w[0].epoch == w[1].epoch && w[1].t - w[0].t > 1.0 {
            gaps.push(serde_json::json!({"device": w[0].device, "epoch": w[0].epoch,
                "from": w[0].t, "to": w[1].t}));
        }
    }
    let mut points = Vec::new();
    for b in &blocks {
        let frame = pkts.iter().find(|p| p.id == b.packet_id).map(|p| p.frame.clone()).unwrap_or_default();
        for (i, pt) in b.points.iter().enumerate() {
            points.push(serde_json::json!({"block": b.id, "idx": i, "p": pt, "frame": frame, "t": b.t}));
        }
    }
    Json(serde_json::json!({"points": points, "gaps": gaps}))
}

async fn poses(State(s): State<Shared>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    let pkts = st.packets(Some("pose")).unwrap_or_default();
    Json(serde_json::json!(pkts.iter().map(|p| serde_json::json!({
        "epoch": p.epoch, "t": p.t, "trans": p.payload.get("trans"),
    })).collect::<Vec<_>>()))
}

#[derive(Deserialize)]
struct ProvQ {
    block: i64,
    idx: usize,
}

async fn provenance(State(s): State<Shared>, Query(q): Query<ProvQ>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    match st.block_by_id(q.block).ok().flatten() {
        Some(b) => Json(serde_json::json!({
            "block": b.id, "packet_id": b.packet_id, "t": b.t,
            "point": b.points.get(q.idx), "chain": b.chain,
        })),
        None => Json(serde_json::json!({"error": "block not found"})),
    }
}

async fn invalidate_info(State(s): State<Shared>) -> Json<serde_json::Value> {
    let st = s.lock().unwrap();
    let blocks = st.blocks().unwrap_or_default();
    Json(serde_json::json!({"complete_blocks": blocks.len()}))
}
