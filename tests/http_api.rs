mod common;

use axum::body::Body;
use axum::Router;
use http_body_util::BodyExt;
use pointcloud_warp::db;
use pointcloud_warp::service::App;
use pointcloud_warp::{demo, web};
use std::sync::Arc;
use tower::ServiceExt;

async fn get(router: &Router, uri: &str) -> (http::StatusCode, String) {
    let resp = router.clone().oneshot(
        http::Request::builder().uri(uri).body(Body::empty()).unwrap()
    ).await.unwrap();
    let status = resp.status();
    let bytes = resp.into_body().collect().await.unwrap().to_bytes();
    (status, String::from_utf8(bytes.to_vec()).unwrap())
}

async fn post_json(router: &Router, uri: &str, json: &str) -> (http::StatusCode, String) {
    let resp = router.clone().oneshot(
        http::Request::builder().method("POST").uri(uri)
            .header("content-type", "application/json")
            .body(Body::from(json.to_string())).unwrap()
    ).await.unwrap();
    let status = resp.status();
    let bytes = resp.into_body().collect().await.unwrap().to_bytes();
    (status, String::from_utf8(bytes.to_vec()).unwrap())
}

fn seeded_router() -> Router {
    let app = Arc::new(App::new(Arc::new(db::open_memory().unwrap())));
    demo::seed(&app);
    web::router(app)
}

#[tokio::test]
async fn index_contains_chinese_title() {
    let router = seeded_router();
    let (status, body) = get(&router, "/").await;
    assert_eq!(status, http::StatusCode::OK);
    assert!(body.contains("点云经纬"), "page must contain 点云经纬");
    assert!(body.contains("/static/app.js"));
}

#[tokio::test]
async fn static_js_is_served() {
    let router = seeded_router();
    let (status, body) = get(&router, "/static/app.js").await;
    assert_eq!(status, http::StatusCode::OK);
    assert!(body.contains("addEventListener"));
}

#[tokio::test]
async fn state_and_points_endpoints_report_data() {
    let router = seeded_router();
    let (s, state) = get(&router, "/api/state").await;
    assert_eq!(s, http::StatusCode::OK);
    let v: serde_json::Value = serde_json::from_str(&state).unwrap();
    assert!(v["map_points"].as_i64().unwrap() > 0);
    assert!(v["blocks_done"].as_i64().unwrap() >= 1);
    assert!(v["blocks_blocked"].as_i64().unwrap() >= 1);

    let (s, points) = get(&router, "/api/points").await;
    assert_eq!(s, http::StatusCode::OK);
    let pv: serde_json::Value = serde_json::from_str(&points).unwrap();
    assert!(pv["frames"].as_array().unwrap().len() >= 3);
}

#[tokio::test]
async fn point_provenance_endpoint() {
    let router = seeded_router();
    let (_, points) = get(&router, "/api/points").await;
    let pv: serde_json::Value = serde_json::from_str(&points).unwrap();
    let id = pv["frames"][0]["points"][0]["id"].as_i64().unwrap();
    let (s, detail) = get(&router, &format!("/api/point/{}", id)).await;
    assert_eq!(s, http::StatusCode::OK);
    let dv: serde_json::Value = serde_json::from_str(&detail).unwrap();
    assert_eq!(dv["chain"].as_array().unwrap().len(), 2);
}

#[tokio::test]
async fn closing_loop_api_returns_422_with_cycle() {
    let router = seeded_router();
    // body->lidar closes a loop with the existing lidar->body calib;
    // a deliberately imperfect inverse leaves a nonzero residual.
    let body = serde_json::json!({
        "name":"calib_lidar_back", "from_frame":"body", "to_frame":"lidar",
        "tx":-0.2,"ty":0.0,"tz":-1.5,"qw":1.0,"qx":0.0,"qy":0.0,"qz":0.0,
        "covariance":[0.0001,0.0,0.0,0.0,0.0,0.0,
                      0.0,0.0001,0.0,0.0,0.0,0.0,
                      0.0,0.0,0.0001,0.0,0.0,0.0,
                      0.0,0.0,0.0,0.0001,0.0,0.0,
                      0.0,0.0,0.0,0.0,0.0001,0.0,
                      0.0,0.0,0.0,0.0,0.0,0.0001]
    }).to_string();
    let (s, txt) = post_json(&router, "/api/edges", &body).await;
    assert_eq!(s, http::StatusCode::UNPROCESSABLE_ENTITY);
    let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
    assert!(v["error"].as_str().unwrap().contains("loop"));
    assert!(v["cycle"]["edge_ids"].as_array().unwrap().len() >= 1);
}

#[tokio::test]
async fn reset_endpoint_reseeds_dataset() {
    let router = seeded_router();
    let (s, _) = post_json(&router, "/api/demo/reset", "{}").await;
    assert_eq!(s, http::StatusCode::OK);
    let (_, state) = get(&router, "/api/state").await;
    let v: serde_json::Value = serde_json::from_str(&state).unwrap();
    assert!(v["streams"].as_i64().unwrap() >= 3);
}
