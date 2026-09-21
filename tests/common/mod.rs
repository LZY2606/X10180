#![allow(dead_code)]
use pointcloud_warp::db;
use pointcloud_warp::math::Mat6;
pub use pointcloud_warp::service::*;
use std::sync::Arc;

pub const T0: i64 = 1_780_000_000_000;

pub fn app() -> Arc<App> {
    Arc::new(App::new(Arc::new(db::open_memory().unwrap())))
}

pub fn cov(scale: f64) -> Option<Vec<f64>> {
    let mut m = Mat6::zero();
    for i in 0..6 {
        m.a[i][i] = scale;
    }
    Some(m.to_flat())
}

pub fn stream(app: &App, id: &str, kind: &str, frame: &str) {
    app.ensure_stream(&StreamInput {
        device_id: id.into(),
        kind: kind.into(),
        coord_frame: frame.into(),
        unit: "m".into(),
    });
}

pub fn lidar_packet(
    app: &App, dt: i64, seq: u32, pts: Vec<[f64; 3]>,
    restart: bool, summary: &str,
) -> i64 {
    app.ingest_packet(&PacketInput {
        device_id: "lidar-A".into(),
        seq,
        seq_modulus: None,
        restart_flag: Some(restart),
        clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(T0 + dt), ..Default::default() },
        coord_frame: "lidar".into(),
        unit: "m".into(),
        content_summary: summary.into(),
        points: pts,
        pose: None,
    })
    .unwrap()
}

pub fn pose_packet(
    app: &App, device: &str, dt: i64, seq: u32, restart: bool,
    x: f64, y: f64, yaw: f64,
) -> i64 {
    let h = yaw / 2.0;
    app.ingest_packet(&PacketInput {
        device_id: device.into(),
        seq,
        seq_modulus: None,
        restart_flag: Some(restart),
        clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(T0 + dt), ..Default::default() },
        coord_frame: "body".into(),
        unit: "m".into(),
        content_summary: "pose".into(),
        points: vec![],
        pose: Some(PoseInput { tx: x, ty: y, tz: 0.0, qw: h.cos(), qx: 0.0, qy: 0.0, qz: h.sin() }),
    })
    .unwrap()
}

pub fn rig(app: &App) {
    stream(app, "lidar-A", "lidar", "lidar");
    stream(app, "gnss-primary", "gnss", "body");
    stream(app, "gnss-backup", "gnss", "body");
    app.add_edge(&EdgeInput {
        name: "calib_lidar".into(),
        from_frame: "lidar".into(),
        to_frame: "body".into(),
        valid_start: None,
        valid_end: None,
        tx: 0.25, ty: 0.0, tz: 1.6,
        qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: cov(2e-4),
    })
    .unwrap();
    app.add_dynamic_edge("pose_primary", "body", "map", "gnss-primary",
        parse(5e-4), None, None).unwrap();
    app.add_dynamic_edge("pose_backup", "body", "map", "gnss-backup",
        parse(5e-3), None, None).unwrap();
}

pub fn parse(scale: f64) -> Mat6 {
    let mut m = Mat6::zero();
    for i in 0..6 {
        m.a[i][i] = scale;
    }
    m
}

pub fn pts(n: i64) -> Vec<[f64; 3]> {
    (0..n).map(|k| [k as f64 * 0.1, 0.5, -0.1]).collect()
}

pub fn map_points_for_frame(app: &App, frame_id: i64) -> Vec<(i64, [f64; 3])> {
    let c = app.db.0.lock().unwrap();
    let mut s = c
        .prepare("SELECT mp.id,mp.x,mp.y,mp.z FROM map_points mp
                  JOIN blocks b ON b.id=mp.block_id
                  JOIN raw_points rp ON rp.id=mp.raw_point_id
                  WHERE b.frame_id=?1 ORDER BY rp.idx")
        .unwrap();
    s.query_map(rusqlite::params![frame_id], |r| {
        Ok((r.get::<_, i64>(0)?, [r.get(1)?, r.get(2)?, r.get(3)?]))
    })
    .unwrap()
    .map(|r| r.unwrap())
    .collect()
}

pub fn frame_ids(app: &App) -> Vec<i64> {
    let c = app.db.0.lock().unwrap();
    let mut s = c.prepare("SELECT id FROM frames ORDER BY time_ms,id").unwrap();
    s.query_map([], |r| r.get::<_, i64>(0)).unwrap().map(|r| r.unwrap()).collect()
}

pub fn assert_close3(a: [f64; 3], b: [f64; 3], tol: f64, ctx: &str) {
    for i in 0..3 {
        assert!(
            (a[i] - b[i]).abs() < tol,
            "{} axis {}: {} vs {} (diff {})",
            ctx, i, a[i], b[i], (a[i] - b[i]).abs()
        );
    }
}
