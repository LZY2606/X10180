//! HTTP 服务：JSON API + 随服务提供的前端静态资源（不下载任何外部地图/瓦片）。

use crate::derive::{invalidate_for_edge, BuildSummary};
use crate::geo::{parse_cov, Se3};
use crate::graph::{add_edge, resolve_path, NewEdge};
use crate::ingest::{IncomingPacket, Ingester};
use crate::db::Db;
use axum::{
    extract::Query,
    http::StatusCode,
    response::{Html, IntoResponse, Json},
    routing::{get, post},
    Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Clone)]
pub struct AppState {
    pub db: Db,
    pub ingester: Arc<Ingester>,
}

pub fn router(db: Db) -> Router {
    let state = AppState {
        db,
        ingester: Arc::new(Ingester::new()),
    };
    Router::new()
        .route("/", get(index))
        .route("/api/stats", get(stats))
        .route("/api/generations", get(generations))
        .route("/api/packets", get(packets))
        .route("/api/gaps", get(gaps))
        .route("/api/points", get(points))
        .route("/api/point/:id", get(point_detail))
        .route("/api/trajectory", get(trajectory))
        .route("/api/edges", get(edges))
        .route("/api/path", get(path))
        .route("/api/events", get(events))
        .route("/api/ingest", post(ingest))
        .route("/api/edge", post(create_edge))
        .route("/api/rebuild", post(rebuild))
        .fallback(get(|| async { (StatusCode::NOT_FOUND, "not found") }))
        .with_state(state)
}

async fn index() -> Html<&'static str> {
    Html(crate::INDEX_HTML)
}

async fn stats(axum::extract::State(s): axum::extract::State<AppState>) -> ApiResult<impl IntoResponse> {
    let v = s.db.run(|db| crate::query::stats(db)).await?;
    Ok(Json(v))
}
async fn generations(axum::extract::State(s): axum::extract::State<AppState>) -> ApiResult<impl IntoResponse> {
    let v = s.db.run(|db| crate::query::generations(db)).await?;
    Ok(Json(v))
}

#[derive(Deserialize)]
struct PacketsQ {
    kind: Option<String>,
}
async fn packets(
    axum::extract::State(s): axum::extract::State<AppState>,
    Query(q): Query<PacketsQ>,
) -> ApiResult<impl IntoResponse> {
    let kind = q.kind.clone();
    let v = s.db.run(move |db| crate::query::packets(db, kind.as_deref())).await?;
    Ok(Json(v))
}

#[derive(Deserialize)]
struct GapsQ {
    #[serde(default = "default_gap")]
    threshold: f64,
}
fn default_gap() -> f64 {
    crate::derive::MAX_POSE_GAP_SEC
}
async fn gaps(
    axum::extract::State(s): axum::extract::State<AppState>,
    Query(q): Query<GapsQ>,
) -> ApiResult<impl IntoResponse> {
    let v = s.db.run(move |db| crate::query::gaps(db, q.threshold)).await?;
    Ok(Json(v))
}

#[derive(Deserialize)]
struct PointsQ {
    #[serde(default = "default_limit")]
    limit: i64,
}
fn default_limit() -> i64 {
    20_000
}
async fn points(
    axum::extract::State(s): axum::extract::State<AppState>,
    Query(q): Query<PointsQ>,
) -> ApiResult<impl IntoResponse> {
    let v = s.db.run(move |db| crate::query::point_cloud(db, q.limit)).await?;
    Ok(Json(v))
}

async fn point_detail(
    axum::extract::State(s): axum::extract::State<AppState>,
    axum::extract::Path(id): axum::extract::Path<i64>,
) -> ApiResult<impl IntoResponse> {
    let v = s.db.run(move |db| crate::query::point_detail(db, id)).await?;
    match v {
        Some(v) => Ok(Json(v)),
        None => Err(ApiError::not_found(format!("派生点 {id} 不存在"))),
    }
}

async fn trajectory(axum::extract::State(s): axum::extract::State<AppState>) -> ApiResult<impl IntoResponse> {
    let v = s.db.run(|db| crate::query::trajectory(db)).await?;
    Ok(Json(v))
}

#[derive(Deserialize)]
struct EdgesQ {
    all: Option<bool>,
}
async fn edges(
    axum::extract::State(s): axum::extract::State<AppState>,
    Query(q): Query<EdgesQ>,
) -> ApiResult<impl IntoResponse> {
    let all = q.all.unwrap_or(false);
    let v = s.db.run(move |db| crate::graph::list_edges(db, all)).await?;
    Ok(Json(v))
}

#[derive(Deserialize)]
struct PathQ {
    source: String,
    target: String,
    at: f64,
}
async fn path(
    axum::extract::State(s): axum::extract::State<AppState>,
    Query(q): Query<PathQ>,
) -> ApiResult<impl IntoResponse> {
    let (src, tgt, at) = (q.source, q.target, q.at);
    let v = s
        .db
        .run(move |db| resolve_path(db, &src, &tgt, at, vec![]))
        .await?;
    Ok(Json(v))
}

#[derive(Deserialize)]
struct EventsQ {
    #[serde(default = "default_event_limit")]
    limit: i64,
}
fn default_event_limit() -> i64 {
    50
}
async fn events(
    axum::extract::State(s): axum::extract::State<AppState>,
    Query(q): Query<EventsQ>,
) -> ApiResult<impl IntoResponse> {
    #[derive(Serialize)]
    struct Ev {
        id: i64,
        at: f64,
        level: String,
        code: String,
        message: String,
    }
    let evs = s
        .db
        .recent_events(q.limit)
        .await?
        .into_iter()
        .map(|(id, at, level, code, message)| Ev {
            id,
            at,
            level,
            code,
            message,
        })
        .collect::<Vec<_>>();
    Ok(Json(evs))
}

async fn ingest(
    axum::extract::State(s): axum::extract::State<AppState>,
    Json(p): Json<IncomingPacket>,
) -> ApiResult<impl IntoResponse> {
    let ing = s.ingester.clone();
    let report = s
        .db
        .run(move |db| ing.ingest(db, p))
        .await?;
    Ok((StatusCode::CREATED, Json(report)))
}

#[derive(Deserialize)]
struct EdgeReq {
    key: String,
    kind: String,
    source_frame: String,
    target_frame: String,
    #[serde(default)]
    q: Option<Vec<f64>>,
    #[serde(default)]
    rot_xyz: Option<Vec<f64>>,
    t: Vec<f64>,
    cov: serde_json::Value,
    valid_from: f64,
    valid_to: f64,
    #[serde(default)]
    rebuild: bool,
}

async fn create_edge(
    axum::extract::State(s): axum::extract::State<AppState>,
    Json(req): Json<EdgeReq>,
) -> ApiResult<impl IntoResponse> {
    if req.t.len() != 3 {
        return Err(ApiError::bad("t 需要 3 个元素"));
    }
    let se3 = match (&req.q, &req.rot_xyz) {
        (Some(q), _) if q.len() == 4 => Se3 {
            q: [q[0], q[1], q[2], q[3]],
            t: [req.t[0], req.t[1], req.t[2]],
        },
        (_, Some(r)) if r.len() == 3 => Se3::from_iso(
            [r[0], r[1], r[2]],
            [req.t[0], req.t[1], req.t[2]],
        ),
        _ => return Err(ApiError::bad("需要 q=[x,y,z,w] 或 rot_xyz=[rx,ry,rz]")),
    };
    let cov = parse_cov(&req.cov).map_err(ApiError::bad)?;
    let do_rebuild = req.rebuild;
    let out = s
        .db
        .run(move |db| {
            let (edge, cycle) = add_edge(
                db,
                NewEdge {
                    key: req.key,
                    kind: req.kind,
                    source_frame: req.source_frame,
                    target_frame: req.target_frame,
                    se3,
                    cov,
                    valid_from: req.valid_from,
                    valid_to: req.valid_to,
                },
            )?;
            let mut invalidated = 0usize;
            let mut rebuild = None;
            if cycle.is_none() {
                invalidated =
                    invalidate_for_edge(db, &edge.key, edge.valid_from, edge.valid_to)?;
                if do_rebuild {
                    rebuild = Some(crate::derive::build_all(db)?);
                }
            }
            anyhow::Ok(EdgeOutcome {
                edge,
                cycle,
                invalidated_blocks: invalidated,
                rebuild,
            })
        })
        .await?;
    let status = if out.cycle.is_some() {
        StatusCode::CONFLICT
    } else {
        StatusCode::CREATED
    };
    Ok((status, Json(out)))
}

#[derive(Serialize)]
struct EdgeOutcome {
    edge: crate::graph::EdgeRecord,
    cycle: Option<crate::graph::CycleError>,
    invalidated_blocks: usize,
    rebuild: Option<BuildSummary>,
}

async fn rebuild(axum::extract::State(s): axum::extract::State<AppState>) -> ApiResult<impl IntoResponse> {
    let summary = tokio::task::spawn_blocking(move || crate::derive::build_all(&s.db))
        .await
        .map_err(|e| ApiError::internal(e.to_string()))??;
    Ok(Json(summary))
}

type ApiResult<T> = Result<T, ApiError>;

struct ApiError {
    status: StatusCode,
    message: String,
}
impl ApiError {
    fn bad(msg: impl Into<String>) -> Self {
        Self {
            status: StatusCode::BAD_REQUEST,
            message: msg.into(),
        }
    }
    fn not_found(msg: String) -> Self {
        Self {
            status: StatusCode::NOT_FOUND,
            message: msg,
        }
    }
    fn internal(msg: String) -> Self {
        Self {
            status: StatusCode::INTERNAL_SERVER_ERROR,
            message: msg,
        }
    }
}
impl IntoResponse for ApiError {
    fn into_response(self) -> axum::response::Response {
        #[derive(Serialize)]
        struct ErrBody {
            error: String,
        }
        (self.status, Json(ErrBody { error: self.message })).into_response()
    }
}
impl From<anyhow::Error> for ApiError {
    fn from(e: anyhow::Error) -> Self {
        ApiError::bad(e.to_string())
    }
}

pub async fn serve(db: Db, addr: &str) -> anyhow::Result<()> {
    let app = router(db);
    let listener = tokio::net::TcpListener::bind(addr).await?;
    println!("点云经纬 服务已启动: http://{addr}");
    axum::serve(listener, app)
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    Ok(())
}

