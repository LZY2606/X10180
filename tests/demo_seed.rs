mod common;
use pointcloud_warp::db;
use pointcloud_warp::demo;
use pointcloud_warp::service::App;
use std::sync::Arc;

#[test]
fn demo_covers_generations_gap_and_two_pose_streams() {
    let app = Arc::new(App::new(Arc::new(db::open_memory().unwrap())));
    demo::seed(&app);
    let c = app.db.0.lock().unwrap();
    let lidar_gens: i64 = c
        .query_row("SELECT COUNT(DISTINCT generation) FROM packets WHERE stream_id=1", [],
                   |r| r.get(0)).unwrap();
    assert!(lidar_gens >= 2, "demo shows multiple generations");
    let blocked: i64 = c
        .query_row("SELECT COUNT(*) FROM blocks WHERE status='blocked'", [], |r| r.get(0))
        .unwrap();
    assert!(blocked >= 1, "gap frame is blocked, not faked");
    let dyn_edges: i64 = c
        .query_row("SELECT COUNT(*) FROM edges WHERE dynamic=1", [], |r| r.get(0)).unwrap();
    assert_eq!(dyn_edges, 2);
}
