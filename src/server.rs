//! Axum HTTP layer.  Frontend assets are embedded in the binary and served
//! by the same process — no external map tiles or point clouds are fetched.

use crate::db::Db;
use crate::demo;
use crate::model::{AppError, PacketInput};
use crate::service::{self, EdgeInput};
use axum::{
    extract::{Path, State},
    http::{header, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use std::sync::Arc;

#[derive(Clone)]
pub struct AppState {
    pub db: Arc<Db>,
}

pub fn router(db: Arc<Db>) -> Router {
    let state = AppState { db };
    Router::new()
        .route("/", get(index))
        .route("/index.html", get(index))
        .route("/static/app.js", get(app_js))
        .route("/static/style.css", get(style_css))
        .route("/api/state", get(get_state))
        .route("/api/point/:id", get(point))
        .route("/api/import", post(import_packets))
        .route("/api/edges", post(create_edge))
        .route("/api/revise", post(revise))
        .route("/api/build", post(build))
        .route("/api/demo/reset", post(reset_demo))
        .with_state(state)
}

async fn index() -> Response {
    asset(include_str!("static/index.html"), "text/html; charset=utf-8")
}
async fn app_js() -> Response {
    asset(include_str!("static/app.js"), "application/javascript; charset=utf-8")
}
async fn style_css() -> Response {
    asset(include_str!("static/style.css"), "text/css; charset=utf-8")
}

fn asset(body: &'static str, kind: &'static str) -> Response {
    (
        StatusCode::OK,
        [(header::CONTENT_TYPE, kind), (header::CACHE_CONTROL, "no-cache")],
        body,
    )
        .into_response()
}

impl IntoResponse for AppError {
    fn into_response(self) -> Response {
        let status = match self.code {
            "cycle_detected" => StatusCode::CONFLICT,
            "singular_covariance" | "unit_error" | "bad_window"
            | "bad_kind" | "bad_time" | "bad_device" | "bad_pose" => {
                StatusCode::UNPROCESSABLE_ENTITY
            }
            "not_found" => StatusCode::NOT_FOUND,
            _ => StatusCode::INTERNAL_SERVER_ERROR,
        };
        (
            status,
            Json(serde_json::json!({"code": self.code, "message": self.message})),
        )
            .into_response()
    }
}

async fn get_state(State(s): State<AppState>) -> Result<Json<serde_json::Value>, AppError> {
    let c = s.db.lock();
    let view = service::state_view(&c)?;
    Ok(Json(serde_json::to_value(view).unwrap()))
}

async fn point(
    State(s): State<AppState>,
    Path(id): Path<i64>,
) -> Result<Json<serde_json::Value>, AppError> {
    let c = s.db.lock();
    let prov = service::point_provenance(&c, id)?;
    Ok(Json(serde_json::to_value(prov).unwrap()))
}

#[derive(serde::Deserialize)]
struct ImportBody {
    packets: Vec<PacketInput>,
    #[serde(default)]
    build: Option<bool>,
}

async fn import_packets(
    State(s): State<AppState>,
    Json(body): Json<ImportBody>,
) -> Result<Json<serde_json::Value>, AppError> {
    let report = {
        let c = s.db.lock();
        service::import_packets(&c, body.packets)?
    };
    let build = if body.build.unwrap_or(true) {
        let c = s.db.lock();
        Some(service::build_all(&c)?)
    } else {
        None
    };
    Ok(Json(serde_json::json!({"import": report, "build": build})))
}

async fn create_edge(
    State(s): State<AppState>,
    Json(input): Json<EdgeInput>,
) -> Result<Json<serde_json::Value>, AppError> {
    let (id, version) = {
        let c = s.db.lock();
        service::add_edge(&c, input)?
    };
    let build = {
        let c = s.db.lock();
        service::build_all(&c)?
    };
    Ok(Json(serde_json::json!({
        "edge_id": id, "version": version, "build": build
    })))
}

async fn revise(
    State(s): State<AppState>,
    Json(input): Json<EdgeInput>,
) -> Result<Json<serde_json::Value>, AppError> {
    let out = {
        let c = s.db.lock();
        service::revise_and_build(&c, input)?
    };
    Ok(Json(out))
}

async fn build(State(s): State<AppState>) -> Result<Json<serde_json::Value>, AppError> {
    let c = s.db.lock();
    let report = service::build_all(&c)?;
    Ok(Json(serde_json::to_value(report).unwrap()))
}

async fn reset_demo(
    State(s): State<AppState>,
) -> Result<Json<serde_json::Value>, AppError> {
    demo::reset(&s.db);
    s.db.set_meta("seeded", "0");
    demo::seed(&s.db);
    let c = s.db.lock();
    let view = service::state_view(&c)?;
    Ok(Json(serde_json::to_value(view).unwrap()))
}
