mod common;
use common::*;

/// Geometry is compared with explicit numeric tolerances, never pixels.
#[test]
fn transformed_points_match_hand_computed_chain() {
    let app = app();
    rig(&app);
    // Identity yaw poses at x = 1.5 at t=100ms bracketing the frame.
    pose_packet(&app, "gnss-primary", 0, 0, true, 1.5, 0.0, 0.0);
    pose_packet(&app, "gnss-primary", 200, 1, false, 1.5, 0.0, 0.0);
    lidar_packet(&app, 100, 0, vec![[1.0, 2.0, 0.0]], true, "single");
    app.recompute();

    let fid = frame_ids(&app)[0];
    let map = map_points_for_frame(&app, fid);
    assert_eq!(map.len(), 1);
    // calib: (1,2,0) -> (1.25, 2.0, 1.6); pose translate x+1.5:
    let expected = [2.75, 2.0, 1.6];
    assert_close3(map[0].1, expected, 1e-9, "composed point");

    // Provenance traceability for the point.
    let detail = app.point_detail(map[0].0).expect("provenance exists");
    assert_eq!(detail.chain.len(), 2);
    assert_eq!(detail.chain[0].edge, "calib_lidar");
    assert!(detail.chain[1].interpolated);
    assert_eq!(detail.edge_versions, vec![1, 1]);
    // Point covariance is positive (noise floor) and symmetric.
    let c = detail.point_covariance;
    assert!(c[0][0] > 0.0 && c[1][1] > 0.0 && c[2][2] > 0.0);
    assert!((c[0][1] - c[1][0]).abs() < 1e-15);
}

#[test]
fn rotation_propagates_to_points_within_tolerance() {
    let app = app();
    rig(&app);
    // 90-degree yaw pose (constant), bracketing frame at 100ms.
    for (dt, seq) in [(0, 0), (200, 1)] {
        let h = std::f64::consts::PI / 4.0;
        app.ingest_packet(&PacketInput {
            device_id: "gnss-primary".into(), seq, seq_modulus: None,
            restart_flag: Some(dt == 0),
            clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(T0 + dt), ..Default::default() },
            coord_frame: "body".into(), unit: "m".into(),
            content_summary: "pose".into(), points: vec![],
            pose: Some(PoseInput { tx: 0.0, ty: 0.0, tz: 0.0,
                qw: h.cos(), qx: 0.0, qy: 0.0, qz: h.sin() }),
        }).unwrap();
    }
    // Raw point (1,0,0) -> body via calib: (1.25,0,1.6). A +90 deg yaw
    // maps (x,y) -> (-y,x), giving (0, 1.25, 1.6).
    lidar_packet(&app, 100, 0, vec![[1.0, 0.0, 0.0]], true, "rot");
    app.recompute();
    let fid = frame_ids(&app)[0];
    let got = map_points_for_frame(&app, fid)[0].1;
    assert_close3(got, [0.0, 1.25, 1.6], 1e-9, "90 degree yaw");
}
