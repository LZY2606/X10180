mod common;
use common::*;

fn setup(app: &std::sync::Arc<App>) {
    for (dt, seq) in [(0, 0), (100, 1), (200, 2)] {
        pose_packet(app, "gnss-primary", dt, seq, dt == 0, dt as f64 * 0.01, 0.0, 0.0);
    }
    lidar_packet(app, 50, 0, pts(4), true, "early");
    lidar_packet(app, 150, 1, pts(4), false, "late");
    app.recompute();
}

#[test]
fn calibration_revision_only_invalidates_covered_window() {
    let app = app();
    rig(&app);
    setup(&app);
    let before: Vec<(i64, String, i64)> = {
        let c = app.db.0.lock().unwrap();
        let mut s = c.prepare("SELECT b.id,b.status,f.time_ms FROM blocks b JOIN frames f ON f.id=b.frame_id ORDER BY f.time_ms").unwrap();
        s.query_map([], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?))).unwrap()
            .map(|r| r.unwrap()).collect()
    };
    assert!(before.iter().all(|(_, st, _)| st == "done"));

    // Revision valid only from T0+100: frame at 50 stays cached, frame at
    // 150 invalidates and rebuilds with version 2.
    let res = app.add_edge(&EdgeInput {
        name: "calib_lidar".into(), from_frame: "lidar".into(), to_frame: "body".into(),
        valid_start: Some(T0 + 100), valid_end: None,
        tx: 0.30, ty: 0.0, tz: 1.6,
        qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: cov(1e-4),
    })
    .unwrap();
    assert_eq!(res.invalidated_blocks, 1);

    let after: Vec<(String, i64, i64)> = {
        let c = app.db.0.lock().unwrap();
        let mut s = c.prepare("SELECT b.status,b.frame_id,f.time_ms FROM blocks b JOIN frames f ON f.id=b.frame_id ORDER BY f.time_ms").unwrap();
        s.query_map([], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?))).unwrap()
            .map(|r| r.unwrap()).collect()
    };
    // Early frame was never queued (only covered frames become pending).
    // recompute rebuilds the covered one; outside stays done.
    app.recompute();
    let versions: Vec<(i64, String, String)> = {
        let c = app.db.0.lock().unwrap();
        let mut s = c.prepare("SELECT f.time_ms,b.status,b.edge_versions FROM blocks b JOIN frames f ON f.id=b.frame_id ORDER BY f.time_ms").unwrap();
        s.query_map([], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?))).unwrap()
            .map(|r| r.unwrap()).collect()
    };
    let _ = after;
    let (early_t, early_st, early_v) = &versions[0];
    let (late_t, late_st, late_v) = &versions[1];
    assert_eq!(*early_t, T0 + 50);
    assert_eq!(early_st, "done");
    assert_eq!(early_v, "[1,1]", "outside window keeps v1 calibration");
    assert_eq!(*late_t, T0 + 150);
    assert_eq!(late_st, "done");
    assert_eq!(late_v, "[2,1]", "covered window uses v2 calibration");
}

#[test]
fn interrupted_building_block_is_never_left_successful() {
    let app = app();
    rig(&app);
    setup(&app);
    // Simulate a crash: flip one done block to building.
    {
        let c = app.db.0.lock().unwrap();
        c.execute("UPDATE blocks SET status='building' WHERE id=1", []).unwrap();
    }
    // A fresh App over the same database recovers it to pending...
    app.recover_blocks();
    {
        let c = app.db.0.lock().unwrap();
        let st: String = c.query_row("SELECT status FROM blocks WHERE id=1", [], |r| r.get(0)).unwrap();
        assert_eq!(st, "pending", "building block recovered to pending");
    }
    // ...and recompute produces a complete done block with derived points.
    app.recompute();
    let (st, n): (String, i64) = {
        let c = app.db.0.lock().unwrap();
        c.query_row(
            "SELECT b.status,(SELECT COUNT(*) FROM map_points mp WHERE mp.block_id=b.id)
             FROM blocks b WHERE b.id=1", [], |r| Ok((r.get(0)?, r.get(1)?)))
            .unwrap()
    };
    assert_eq!(st, "done");
    assert_eq!(n, 4);
}

#[test]
fn recompute_restarts_from_complete_blocks() {
    let app = app();
    rig(&app);
    setup(&app);
    // Second recompute must not duplicate derived points (idempotent).
    let r1: i64 = { let c = app.db.0.lock().unwrap();
        c.query_row("SELECT COUNT(*) FROM map_points", [], |r| r.get(0)).unwrap() };
    app.recompute();
    let r2: i64 = { let c = app.db.0.lock().unwrap();
        c.query_row("SELECT COUNT(*) FROM map_points", [], |r| r.get(0)).unwrap() };
    assert_eq!(r1, r2);
}
