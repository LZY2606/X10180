//! HTTP API and embedded static frontend.

use crate::service::{App, EdgeInput, PacketInput};
use axum::{
    extract::{Path, Query, State},
    http::{header, StatusCode},
    response::{Html, IntoResponse, Json},
    routing::{get, post},
    Router,
};
use rusqlite::params;
use serde::Deserialize;
use serde_json::{json, Value};
use std::sync::Arc;

pub fn router(app: Arc<App>) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/static/app.js", get(app_js))
        .route("/api/state", get(state))
        .route("/api/streams", get(streams))
        .route("/api/packets", get(packets))
        .route("/api/edges", get(edges).post(add_edge))
        .route("/api/ingest", post(ingest))
        .route("/api/recompute", post(recompute))
        .route("/api/blocks", get(blocks))
        .route("/api/points", get(points))
        .route("/api/point/{id}", get(point_detail))
        .route("/api/path", get(path_query))
        .route("/api/trajectory", get(trajectory))
        .route("/api/demo/reset", post(reset_demo))
        .with_state(app)
}

async fn index() -> impl IntoResponse {
    (
        [(header::CONTENT_TYPE, "text/html; charset=utf-8")],
        Html(INDEX_HTML),
    )
}

async fn app_js() -> impl IntoResponse {
    (
        [(header::CONTENT_TYPE, "application/javascript; charset=utf-8")],
        APP_JS,
    )
}

async fn state(State(app): State<Arc<App>>) -> Json<Value> {
    let c = app.db.0.lock().unwrap();
    let one = |sql: &str| -> i64 {
        c.query_row(sql, [], |r| r.get(0)).unwrap_or(0)
    };
    Json(json!({
        "streams": one("SELECT COUNT(*) FROM streams"),
        "packets": one("SELECT COUNT(*) FROM packets"),
        "frames": one("SELECT COUNT(*) FROM frames"),
        "poses": one("SELECT COUNT(*) FROM poses"),
        "edges": one("SELECT COUNT(*) FROM edges"),
        "blocks_done": one("SELECT COUNT(*) FROM blocks WHERE status='done'"),
        "blocks_blocked": one("SELECT COUNT(*) FROM blocks WHERE status='blocked'"),
        "blocks_pending": one("SELECT COUNT(*) FROM blocks WHERE status IN ('pending','building','stale')"),
        "map_points": one("SELECT COUNT(*) FROM map_points"),
    }))
}

async fn streams(State(app): State<Arc<App>>) -> Json<Value> {
    let c = app.db.0.lock().unwrap();
    let mut s = c
        .prepare("SELECT id,device_id,kind,coord_frame,unit FROM streams ORDER BY id")
        .unwrap();
    let rows = s
        .query_map([], |r| {
            Ok(json!({
                "id": r.get::<_, i64>(0)?,
                "device_id": r.get::<_, String>(1)?,
                "kind": r.get::<_, String>(2)?,
                "coord_frame": r.get::<_, String>(3)?,
                "unit": r.get::<_, String>(4)?,
            }))
        })
        .unwrap()
        .map(|r| r.unwrap())
        .collect::<Vec<_>>();
    Json(json!(rows))
}

async fn packets(State(app): State<Arc<App>>) -> Json<Value> {
    let c = app.db.0.lock().unwrap();
    let mut s = c
        .prepare(
            "SELECT p.id,s.device_id,p.generation,p.seq,p.gps_ms,p.raw_clock,p.raw_ts,
                    p.coord_frame,p.unit,p.content_summary,p.flag,p.received_order
             FROM packets p JOIN streams s ON s.id=p.stream_id
             ORDER BY p.received_order",
        )
        .unwrap();
    let rows = s
        .query_map([], |r| {
            Ok(json!({
                "id": r.get::<_, i64>(0)?,
                "device_id": r.get::<_, String>(1)?,
                "generation": r.get::<_, i64>(2)?,
                "seq": r.get::<_, i64>(3)?,
                "gps_ms": r.get::<_, i64>(4)?,
                "raw_clock": r.get::<_, String>(5)?,
                "raw_ts": r.get::<_, String>(6)?,
                "coord_frame": r.get::<_, String>(7)?,
                "unit": r.get::<_, String>(8)?,
                "content_summary": r.get::<_, String>(9)?,
                "flag": r.get::<_, String>(10)?,
                "received_order": r.get::<_, i64>(11)?,
            }))
        })
        .unwrap()
        .map(|r| r.unwrap())
        .collect::<Vec<_>>();
    Json(json!(rows))
}

async fn edges(State(app): State<Arc<App>>) -> Json<Value> {
    Json(json!(app.list_edges()))
}

async fn add_edge(
    State(app): State<Arc<App>>,
    Json(input): Json<EdgeInput>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    match app.add_edge(&input) {
        Ok(r) => Ok(Json(json!(r))),
        Err(cyc) => Err((
            StatusCode::UNPROCESSABLE_ENTITY,
            Json(json!({ "error": cyc.error, "cycle": cyc.cycle })),
        )),
    }
}

async fn ingest(
    State(app): State<Arc<App>>,
    Json(input): Json<PacketInput>,
) -> Result<Json<Value>, (StatusCode, String)> {
    match app.ingest_packet(&input) {
        Ok(id) => {
            let report = app.recompute();
            Ok(Json(json!({ "packet_id": id, "build": report })))
        }
        Err(e) => Err((StatusCode::BAD_REQUEST, e)),
    }
}

async fn recompute(State(app): State<Arc<App>>) -> Json<Value> {
    Json(json!(app.recompute()))
}

async fn blocks(State(app): State<Arc<App>>) -> Json<Value> {
    let c = app.db.0.lock().unwrap();
    let mut s = c
        .prepare(
            "SELECT b.id,b.frame_id,b.source_frame,b.target_frame,b.status,b.error,b.edge_versions,
                    f.time_ms,f.generation
             FROM blocks b JOIN frames f ON f.id=b.frame_id ORDER BY f.time_ms,b.id",
        )
        .unwrap();
    let rows = s
        .query_map([], |r| {
            Ok(json!({
                "block_id": r.get::<_, i64>(0)?,
                "frame_id": r.get::<_, i64>(1)?,
                "source_frame": r.get::<_, String>(2)?,
                "target_frame": r.get::<_, String>(3)?,
                "status": r.get::<_, String>(4)?,
                "error": r.get::<_, Option<String>>(5)?,
                "edge_versions": r.get::<_, String>(6)?,
                "time_ms": r.get::<_, i64>(7)?,
                "generation": r.get::<_, i64>(8)?,
            }))
        })
        .unwrap()
        .map(|r| r.unwrap())
        .collect::<Vec<_>>();
    Json(json!(rows))
}

async fn points(State(app): State<Arc<App>>) -> Json<Value> {
    let c = app.db.0.lock().unwrap();
    // Return points grouped by frame with raw and map coordinates and the
    // frame time so the browser can color by frame and spot time gaps.
    let mut s = c
        .prepare(
            "SELECT f.id,f.time_ms,f.generation,b.status,mp.id,mp.x,mp.y,mp.z,
                    rp.x,rp.y,rp.z
             FROM frames f
             JOIN blocks b ON b.frame_id=f.id
             LEFT JOIN map_points mp ON mp.block_id=b.id
             JOIN raw_points rp ON rp.frame_id=f.id AND rp.id=mp.raw_point_id
             WHERE f.canonical=1
             ORDER BY f.time_ms,rp.idx",
        )
        .unwrap();
    let mut frames: Vec<Value> = Vec::new();
    let mut current: Option<(i64, i64, i64, String, Vec<Value>)> = None;
    let rows = s
        .query_map([], |r| {
            Ok((
                r.get::<_, i64>(0)?,
                r.get::<_, i64>(1)?,
                r.get::<_, i64>(2)?,
                r.get::<_, String>(3)?,
                r.get::<_, Option<i64>>(4)?,
                r.get::<_, Option<f64>>(5)?,
                r.get::<_, Option<f64>>(6)?,
                r.get::<_, Option<f64>>(7)?,
                r.get::<_, f64>(8)?,
                r.get::<_, f64>(9)?,
                r.get::<_, f64>(10)?,
            ))
        })
        .unwrap();
    for r in rows {
        let (fid, t, gen, status, mpid, mx, my, mz, rx, ry, rz) = r.unwrap();
        if current.as_ref().map_or(true, |(id, ..)| *id != fid) {
            if let Some(v) = current.take() {
                frames.push(frame_json(v));
            }
            current = Some((fid, t, gen, status, Vec::new()));
        }
        if let (Some(mpid), Some(mx), Some(my), Some(mz)) = (mpid, mx, my, mz) {
            current.as_mut().unwrap().4.push(json!({
                "id": mpid, "map": [mx,my,mz], "raw": [rx,ry,rz]
            }));
        }
    }
    if let Some(v) = current {
        frames.push(frame_json(v));
    }
    Json(json!({ "frames": frames }))
}

fn frame_json(v: (i64, i64, i64, String, Vec<Value>)) -> Value {
    json!({
        "frame_id": v.0, "time_ms": v.1, "generation": v.2,
        "status": v.3, "points": v.4
    })
}

async fn point_detail(
    State(app): State<Arc<App>>,
    Path(id): Path<i64>,
) -> Result<Json<Value>, StatusCode> {
    app.point_detail(id)
        .map(|d| Json(json!(d)))
        .ok_or(StatusCode::NOT_FOUND)
}

#[derive(Deserialize)]
struct PathQ {
    from: String,
    to: String,
    time: i64,
}

async fn path_query(
    State(app): State<Arc<App>>,
    Query(q): Query<PathQ>,
) -> Result<Json<Value>, (StatusCode, String)> {
    app.paths_at(&q.from, &q.to, q.time)
        .map(|p| Json(serde_json::to_value(p).unwrap()))
        .map_err(|e| (StatusCode::NOT_FOUND, e))
}

async fn trajectory(State(app): State<Arc<App>>) -> Json<Value> {
    let c = app.db.0.lock().unwrap();
    let mut s = c
        .prepare(
            "SELECT s.device_id,p.generation,p.time_ms,p.tx,p.ty,p.tz
             FROM poses p JOIN streams s ON s.id=p.stream_id
             ORDER BY s.device_id,p.time_ms",
        )
        .unwrap();
    let rows = s
        .query_map([], |r| {
            Ok(json!({
                "device": r.get::<_, String>(0)?,
                "generation": r.get::<_, i64>(1)?,
                "time_ms": r.get::<_, i64>(2)?,
                "t": [r.get::<_, f64>(3)?, r.get::<_, f64>(4)?, r.get::<_, f64>(5)?],
            }))
        })
        .unwrap()
        .map(|r| r.unwrap())
        .collect::<Vec<_>>();
    Json(json!({ "poses": rows }))
}

async fn reset_demo(State(app): State<Arc<App>>) -> Json<Value> {
    {
        let c = app.db.0.lock().unwrap();
        for table in [
            "map_points", "blocks", "poses", "raw_points", "frames", "packets", "edges", "streams",
        ] {
            c.execute(&format!("DELETE FROM {}", table), params![]).unwrap();
        }
    }
    crate::demo::seed(&app);
    Json(json!({ "ok": true }))
}

const INDEX_HTML: &str = include_str!("../static/index.html");
const APP_JS: &str = include_str!("../static/app.js");
