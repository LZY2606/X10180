mod common;
use common::*;
use pointcloud_warp::graph::{interpolate_pose, InterpError, PoseSample};
use pointcloud_warp::math::Rigid;

fn sample(t: i64, gen: i64, x: f64) -> PoseSample {
    PoseSample { time_ms: t, generation: gen,
        transform: Rigid { t: [x, 0.0, 0.0], q: [1.0, 0.0, 0.0, 0.0] } }
}

#[test]
fn interpolation_never_crosses_generation() {
    let s = vec![sample(0, 1, 0.0), sample(100, 2, 1.0)];
    let err = interpolate_pose(&s, 50, 250).unwrap_err();
    assert!(matches!(err, InterpError::CrossesGeneration { left_gen: 1, right_gen: 2 }));
}

#[test]
fn interpolation_rejects_oversized_gap_and_extrapolation() {
    let s = vec![sample(0, 1, 0.0), sample(500, 1, 5.0)];
    assert!(matches!(
        interpolate_pose(&s, 250, 250),
        Err(InterpError::IntervalTooLarge { gap_ms: 500, max_ms: 250 })
    ));
    assert!(matches!(interpolate_pose(&s, -1, 250), Err(InterpError::OutOfRange { .. })));
    assert!(matches!(interpolate_pose(&s, 501, 250), Err(InterpError::OutOfRange { .. })));
}

#[test]
fn interpolation_is_linear_within_bounds() {
    let s = vec![sample(0, 1, 0.0), sample(100, 1, 1.0), sample(200, 1, 3.0)];
    let p = interpolate_pose(&s, 50, 250).unwrap();
    assert!((p.transform.t[0] - 0.5).abs() < 1e-12);
    let p = interpolate_pose(&s, 150, 250).unwrap();
    assert!((p.transform.t[0] - 2.0).abs() < 1e-12);
    // Exact sample.
    let p = interpolate_pose(&s, 100, 250).unwrap();
    assert!((p.transform.t[0] - 1.0).abs() < 1e-12);
}

#[test]
fn frame_in_generation_gap_is_blocked_then_unaffected_frame_builds() {
    let app = app();
    rig(&app);
    // gen 1 poses 0..200ms; gen 2 poses 4000..4100ms.
    for (dt, seq, restart) in [(0, 0, true), (100, 1, false), (200, 2, false)] {
        pose_packet(&app, "gnss-primary", dt, seq, restart, dt as f64 * 0.01, 0.0, 0.0);
    }
    for (dt, seq) in [(4000, 0), (4100, 1)] {
        pose_packet(&app, "gnss-primary", dt, seq, dt == 4000, 0.0, 0.0, 0.0);
    }
    // Lidar frame at 50ms builds; frame at 2500ms sits in the gap and
    // cannot interpolate across generations.
    lidar_packet(&app, 50, 0, pts(2), true, "inside gen1");
    lidar_packet(&app, 2500, 0, pts(2), true, "inside gap");
    let report = app.recompute();
    assert!(report.built >= 1);
    let statuses: Vec<(String, Option<String>)> = {
        let c = app.db.0.lock().unwrap();
        let mut s = c.prepare("SELECT status,error FROM blocks ORDER BY id").unwrap();
        s.query_map([], |r| Ok((r.get(0)?, r.get(1)?))).unwrap()
            .map(|r| r.unwrap()).collect()
    };
    assert_eq!(statuses[0].0, "done");
    assert_eq!(statuses[1].0, "blocked");
}
