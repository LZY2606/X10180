mod common;
use common::*;

#[test]
fn singular_covariance_is_rejected() {
    // Direct validation: translation block with zero eigenvalues.
    let mut bad = vec![0.0; 36];
    bad[0] = 1e-20;
    bad[3 * 6 + 3] = 0.01;
    bad[4 * 6 + 4] = 0.01;
    bad[5 * 6 + 5] = 0.01;
    let m = pointcloud_warp::math::Mat6::from_flat(&bad);
    let e = pointcloud_warp::math::validate_covariance(&m).unwrap_err();
    assert!(e.contains("singular"), "got: {}", e);

    // End-to-end through edge insertion.
    let app = app();
    let err = app.add_edge(&EdgeInput {
        name: "bad".into(), from_frame: "a".into(), to_frame: "b".into(),
        valid_start: None, valid_end: None,
        tx: 0.0, ty: 0.0, tz: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: Some(bad),
    });
    assert!(err.unwrap_err().error.contains("singular"));
}

#[test]
fn nonsymmetric_covariance_is_rejected() {
    let app = app();
    let mut bad = vec![0.0; 36];
    for i in 0..6 { bad[i * 6 + i] = 0.01; }
    bad[1] = 0.005; // asymmetric off-diagonal
    let err = app.add_edge(&EdgeInput {
        name: "asym".into(), from_frame: "a".into(), to_frame: "b".into(),
        valid_start: None, valid_end: None,
        tx: 0.0, ty: 0.0, tz: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: Some(bad),
    });
    assert!(err.unwrap_err().error.contains("not symmetric"));
}

#[test]
fn wrong_shape_covariance_is_rejected() {
    let app = app();
    let err = app.add_edge(&EdgeInput {
        name: "shape".into(), from_frame: "a".into(), to_frame: "b".into(),
        valid_start: None, valid_end: None,
        tx: 0.0, ty: 0.0, tz: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: Some(vec![1.0; 10]),
    });
    assert!(err.unwrap_err().error.contains("36 numbers"));
}

#[test]
fn unit_and_frame_mismatch_are_rejected() {
    let app = app();
    stream(&app, "lidar-A", "lidar", "lidar");
    let wrong_unit = app.ingest_packet(&PacketInput {
        device_id: "lidar-A".into(), seq: 0, seq_modulus: None,
        restart_flag: Some(true),
        clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(T0), ..Default::default() },
        coord_frame: "lidar".into(), unit: "fathom".into(),
        content_summary: "x".into(), points: vec![], pose: None,
    });
    assert!(wrong_unit.unwrap_err().contains("unknown unit"));

    let wrong_frame = app.ingest_packet(&PacketInput {
        device_id: "lidar-A".into(), seq: 0, seq_modulus: None,
        restart_flag: Some(true),
        clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(T0), ..Default::default() },
        coord_frame: "camera".into(), unit: "m".into(),
        content_summary: "x".into(), points: vec![], pose: None,
    });
    assert!(wrong_frame.unwrap_err().contains("coordinate frame mismatch"));
}

#[test]
fn millimetre_points_are_normalized_to_metres_but_raw_unit_recorded() {
    let app = app();
    app.ensure_stream(&StreamInput {
        device_id: "mm-lidar".into(), kind: "lidar".into(),
        coord_frame: "lidar".into(), unit: "mm".into(),
    });
    app.ingest_packet(&PacketInput {
        device_id: "mm-lidar".into(), seq: 0, seq_modulus: None,
        restart_flag: Some(true),
        clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(T0), ..Default::default() },
        coord_frame: "lidar".into(), unit: "mm".into(),
        content_summary: "1000 mm point".into(),
        points: vec![[1000.0, 2000.0, 0.0]], pose: None,
    })
    .unwrap();
    let c = app.db.0.lock().unwrap();
    let (x, y, unit): (f64, f64, String) = c
        .query_row("SELECT rp.x,rp.y,f.unit FROM raw_points rp JOIN frames f ON f.id=rp.frame_id",
                   [], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)))
        .unwrap();
    assert!((x - 1.0).abs() < 1e-12);
    assert!((y - 2.0).abs() < 1e-12);
    assert_eq!(unit, "mm");
}

#[test]
fn dbg_singular_message() {
    use common::*;
    let app = app();
    let mut bad = vec![0.0; 36];
    bad[0] = 1e-20;
    bad[15] = 0.01;
    let r = app.add_edge(&EdgeInput {
        name: "dbg".into(), from_frame: "a".into(), to_frame: "b".into(),
        valid_start: None, valid_end: None,
        tx:0.0,ty:0.0,tz:0.0,qw:1.0,qx:0.0,qy:0.0,qz:0.0,
        covariance: Some(bad) });
    match r { Ok(v)=>println!("OK {:?}",v), Err(e)=>println!("ERR {}", e.error) }
}
