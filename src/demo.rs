//! Built-in deterministic dataset so the browser view has content
//! without downloading any maps or point clouds.

use crate::service::{App, EdgeInput, PacketInput, ClockInput, PoseInput, StreamInput};
use crate::math::Mat6;

fn cov(scale: f64) -> Vec<f64> {
    let mut m = Mat6::zero();
    for i in 0..6 {
        m.a[i][i] = scale;
    }
    m.to_flat()
}

pub fn seed(app: &App) {
    let c = app.db.0.lock().unwrap();
    let n: i64 = c.query_row("SELECT COUNT(*) FROM streams", [], |r| r.get(0)).unwrap();
    drop(c);
    if n > 0 {
        return;
    }

    app.ensure_stream(&StreamInput {
        device_id: "lidar-A".into(),
        kind: "lidar".into(),
        coord_frame: "lidar".into(),
        unit: "m".into(),
    });
    app.ensure_stream(&StreamInput {
        device_id: "gnss-primary".into(),
        kind: "gnss".into(),
        coord_frame: "body".into(),
        unit: "m".into(),
    });
    app.ensure_stream(&StreamInput {
        device_id: "gnss-backup".into(),
        kind: "gnss".into(),
        coord_frame: "body".into(),
        unit: "m".into(),
    });

    // Choose a base time near a GPS 10-bit week boundary region but safely
    // in the current rollover epoch.
    let base = crate::time::utc_ymd_hms_ms(2026, 9, 20, 10, 0, 0, 0);

    // Static sensor->body calibration (two edges, acyclic graph).
    let _ = app.add_edge(&EdgeInput {
        name: "calib_lidar".into(),
        from_frame: "lidar".into(),
        to_frame: "body".into(),
        valid_start: None,
        valid_end: None,
        tx: 0.25, ty: 0.0, tz: 1.6,
        qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: Some(cov(2e-4)),
    });
    let _ = app.add_dynamic_edge("pose_primary", "body", "map", "gnss-primary",
        parse_cov(5e-4), None, None);
    let _ = app.add_dynamic_edge("pose_backup", "body", "map", "gnss-backup",
        parse_cov(5e-3), None, None);

    // Two pose generations with a visible time gap (>2 s).
    let samples: [(i64, f64, f64, f64); 3] = [
        (0, 0.0, 0.0, 0.0),
        (100, 0.2, 0.05, 0.002),
        (200, 0.4, 0.10, 0.004),
    ];
    for (dt, x, y, yaw) in samples {
        let half = yaw / 2.0_f64;
        app.ingest_packet(&PacketInput {
            device_id: "gnss-primary".into(),
            seq: dt as u32,
            seq_modulus: None,
            restart_flag: Some(dt == 0),
            clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(base + dt as i64), ..Default::default() },
            coord_frame: "body".into(), unit: "m".into(),
            content_summary: format!("primary pose +{}ms", dt),
            points: vec![],
            pose: Some(PoseInput { tx: x, ty: y, tz: 0.0, qw: half.cos(), qx: 0.0, qy: 0.0, qz: half.sin() }),
        }).unwrap();
        // Backup path runs parallel with noticeably worse precision.
        app.ingest_packet(&PacketInput {
            device_id: "gnss-backup".into(),
            seq: dt as u32,
            seq_modulus: None,
            restart_flag: Some(dt == 0),
            clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(base + dt as i64), ..Default::default() },
            coord_frame: "body".into(), unit: "m".into(),
            content_summary: format!("backup pose +{}ms", dt),
            points: vec![],
            pose: Some(PoseInput { tx: x + 0.01, ty: y - 0.01, tz: 0.0, qw: half.cos(), qx: 0.0, qy: 0.0, qz: half.sin() }),
        }).unwrap();
    }
    // Second generation after a 4 s gap.
    let gen2 = base + 4_000;
    for (i, (dt, x, y, yaw)) in [
        (0, 1.0_f64, 0.2_f64, 0.01_f64),
        (100, 1.2, 0.22, 0.012),
    ].iter().enumerate() {
        let h = yaw / 2.0;
        for (dev, dx, scale) in [("gnss-primary", 0.0, 1), ("gnss-backup", 0.01, 1)] {
            app.ingest_packet(&PacketInput {
                device_id: dev.into(),
                seq: *dt as u32,
                seq_modulus: None,
                restart_flag: Some(i == 0),
                clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(gen2 + *dt as i64), ..Default::default() },
                coord_frame: "body".into(), unit: "m".into(),
                content_summary: "pose gen2".into(),
                points: vec![],
                pose: Some(PoseInput { tx: x + dx, ty: y - dx, tz: 0.0,
                    qw: h.cos(), qx: 0.0, qy: 0.0, qz: h.sin() * scale as f64 }),
            }).unwrap();
        }
    }

    // Sparse LiDAR frames. Frame 0 and 100 derive; frame at 200ms has no
    // bracketing poses past 200 (edge case handled gracefully as in-range
    // exact sample). A frame inside the 4 s gap becomes blocked (no
    // interpolation across the generation gap).
    let frames = [50i64, 150, 200, 2_500, 4_050];
    for (i, dt) in frames.iter().enumerate() {
        let mut points = Vec::new();
        for k in 0..24 {
            let a = (k as f64) * 0.5 + (i as f64) * 0.07;
            let r = 2.0 + 0.35 * (k as f64 % 5.0);
            points.push([r * a.cos(), r * a.sin(), 0.1 * ((k % 3) as f64) - 0.1]);
        }
        app.ingest_packet(&PacketInput {
            device_id: "lidar-A".into(),
            seq: i as u32,
            seq_modulus: None,
            restart_flag: Some(i == 0 || *dt == 4_050),
            clock: ClockInput { kind: "gps_ms".into(), gps_ms: Some(base + dt), ..Default::default() },
            coord_frame: "lidar".into(), unit: "m".into(),
            content_summary: format!("sparse scan {} (24 pts)", i),
            points,
            pose: None,
        }).unwrap();
    }

    app.recompute();
}

fn parse_cov(scale: f64) -> Mat6 {
    let mut m = Mat6::zero();
    for i in 0..6 {
        m.a[i][i] = scale;
    }
    m
}
