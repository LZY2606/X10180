//! Deterministic built-in dataset so `cargo run` immediately shows a
//! multi-generation frame chain, a time gap, calibration versions and a
//! route with candidates.  No external map or point cloud is downloaded.

use crate::model::PacketInput;
use crate::service::EdgeInput;
use crate::{db::Db, service};

pub fn is_seeded(db: &Db) -> bool {
    db.meta("seeded").as_deref() == Some("1")
}

pub fn reset(db: &Db) {
    let c = db.lock();
    c.execute_batch(
        "DELETE FROM derived_points; DELETE FROM blocks; DELETE FROM poses;
         DELETE FROM edges; DELETE FROM packets; DELETE FROM generations;
         DELETE FROM frames;",
    )
    .unwrap();
}

pub fn seed(db: &Db) {
    if is_seeded(db) {
        return;
    }
    {
        let c = db.lock();
        seed_inner(&c);
    }
    db.set_meta("seeded", "1");
}

const DEV: &str = "survey-1";
const T0: f64 = 1_000.0;

fn gnss(seq: i64, t: f64, x: f64, y: f64, yaw: f64, marker: &str) -> PacketInput {
    PacketInput {
        kind: "gnss".into(),
        device_id: DEV.into(),
        seq,
        boot_id: Some(if marker == "A" { 1 } else { 2 }),
        boot_marker: Some(marker.into()),
        timestamp: t,
        week: None,
        leap_second_flag: false,
        time_scale: Some("gps".into()),
        coord_frame: Some(format!("body/{DEV}")),
        length_unit: Some("m".into()),
        angle_unit: Some("rad".into()),
        px: Some(x),
        py: Some(y),
        pz: Some(0.0),
        roll: Some(0.0),
        pitch: Some(0.0),
        yaw: Some(yaw),
        quat: None,
        cov: Some(vec![
            4e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 4e-4, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9e-4, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 2.5e-5,
        ]),
        points: None,
    }
}

fn lidar(seq: i64, t: f64, marker: &str, local: Vec<[f64; 3]>) -> PacketInput {
    PacketInput {
        kind: "lidar".into(),
        device_id: DEV.into(),
        seq,
        boot_id: Some(if marker == "A" { 1 } else { 2 }),
        boot_marker: Some(marker.into()),
        timestamp: t,
        week: None,
        leap_second_flag: false,
        time_scale: Some("gps".into()),
        coord_frame: Some(format!("lidar/{DEV}")),
        length_unit: Some("m".into()),
        angle_unit: Some("rad".into()),
        px: None,
        py: None,
        pz: None,
        roll: None,
        pitch: None,
        yaw: None,
        quat: None,
        cov: None,
        points: Some(
            local
                .into_iter()
                .map(|p| crate::model::RawPoint {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                    intensity: None,
                })
                .collect(),
        ),
    }
}

struct Pose {
    x: f64,
    y: f64,
    yaw: f64,
}

/// Body trajectory at continuous time t (generation A).  Moves along +x with
/// a slow turn; landmark points are generated in map frame and transformed
/// back into the sensor frame so the demo is geometrically exact.
fn pose_a(t: f64) -> Pose {
    let u = (t - T0) / 10.0;
    Pose {
        x: 0.5 * u * 10.0,
        y: (u * 1.2).sin() * 0.3,
        yaw: 0.05 * u * 10.0,
    }
}

fn rot2(yaw: f64, p: [f64; 2]) -> [f64; 2] {
    let (c, s) = (yaw.cos(), yaw.sin());
    [c * p[0] - s * p[1], s * p[0] + c * p[1]]
}

/// Transform a fixed map landmark into the lidar sensor frame given body pose
/// and the lidar->body extrinsic (q, t_lidar_in_body).
fn map_to_sensor(
    map: [f64; 3],
    pose: &Pose,
    body_from_lidar: [f64; 3],
    lidar_yaw_in_body: f64,
) -> [f64; 3] {
    // map -> body: inverse of body pose (translation then 2D rotation)
    let d = [map[0] - pose.x, map[1] - pose.y, map[2]];
    let inv_xy = rot2(-pose.yaw, [d[0], d[1]]);
    let inv = [inv_xy[0], inv_xy[1], d[2]];
    // body -> lidar: inverse of lidar->body
    let d2 = [
        inv[0] - body_from_lidar[0],
        inv[1] - body_from_lidar[1],
        inv[2] - body_from_lidar[2],
    ];
    let s3 = rot2(-lidar_yaw_in_body, [d2[0], d2[1]]);
    [s3[0], s3[1], d2[2]]
}

fn landmarks() -> Vec<[f64; 3]> {
    let mut v = Vec::new();
    // two rows of "fence posts" alongside the trajectory
    for i in 0..24 {
        let x = i as f64 * 0.25;
        v.push([x, 1.2, 0.0]);
        v.push([x, -1.2, 0.0]);
        if i % 3 == 0 {
            v.push([x, 0.0, 0.4]);
        }
    }
    // a sparse arc of points
    for k in 0..12 {
        let a = (k as f64 / 12.0) * std::f64::consts::PI;
        v.push([2.0 + a.cos() * 0.8, 0.0, a.sin() * 0.8]);
    }
    v
}

const LB: [f64; 3] = [0.12, 0.0, -0.05];
const LYAW: f64 = 0.02;

fn cov_diag(ts: [f64; 3], rs: [f64; 3]) -> Vec<f64> {
    let mut m = vec![0.0; 36];
    for i in 0..3 {
        m[i * 6 + i] = ts[i];
        m[(i + 3) * 6 + (i + 3)] = rs[i];
    }
    m
}

fn edge(
    source: &str,
    target: &str,
    t: [f64; 3],
    yaw: f64,
    ts: [f64; 3],
    rs: [f64; 3],
    from: Option<f64>,
    to: Option<f64>,
) -> EdgeInput {
    let q = crate::math::Quat::from_axis_angle([0.0, 0.0, 1.0], yaw);
    EdgeInput {
        source: source.into(),
        target: target.into(),
        valid_from: from,
        valid_to: to,
        translation: t,
        quat: Some(q.0),
        euler_rpy: None,
        covariance: cov_diag(ts, rs),
        origin: Some("calibration".into()),
    }
}

fn seed_inner(c: &rusqlite::Connection) {
    let mut packets: Vec<PacketInput> = Vec::new();
    let lm = landmarks();
    let mut seq_g = 0;
    let mut seq_l = 1000;

    // ---- Generation A: t = T0..T0+10, 2 Hz GNSS / 2 Hz LiDAR -------------
    // Deliberately leave a pose gap between t=4.5 and t=5.6 (> MAX_INTERP_GAP)
    // so points in that window show the interpolation guard.
    let gnss_times: Vec<f64> = vec![
        T0,
        T0 + 0.5,
        T0 + 1.0,
        T0 + 1.5,
        T0 + 2.0,
        T0 + 2.5,
        T0 + 3.0,
        T0 + 3.5,
        T0 + 4.0,
        T0 + 4.5,
        // gap
        T0 + 5.6,
        T0 + 6.1,
        T0 + 6.6,
        T0 + 7.1,
        T0 + 7.6,
        T0 + 8.1,
        T0 + 8.6,
        T0 + 9.1,
        T0 + 9.6,
    ];
    for (i, t) in gnss_times.iter().enumerate() {
        seq_g += 1;
        let p = pose_a(*t);
        packets.push(gnss(seq_g, *t, p.x, p.y, p.yaw, "A"));
        let _ = i;
    }

    let lidar_times: Vec<f64> = (0..20)
        .map(|i| T0 + 0.25 + i as f64 * 0.5)
        .collect();
    for (i, t) in lidar_times.iter().enumerate() {
        seq_l += 1;
        let pose = pose_a(*t);
        let local = lm
            .iter()
            .enumerate()
            .filter(|(k, _)| (k + i) % 4 == 0)
            .map(|(_, m)| map_to_sensor(*m, &pose, LB, LYAW))
            .collect();
        packets.push(lidar(seq_l, *t, "A", local));
    }

    // Late duplicate: exactly re-send the earlier lidar packet seq=1003
    // (t=T0+1.25, point selection offset i=2) after later packets arrived.
    {
        let late_t = T0 + 1.25;
        let pose = pose_a(late_t);
        let local = lm
            .iter()
            .enumerate()
            .filter(|(k, _)| (k + 2) % 4 == 0)
            .map(|(_, m)| map_to_sensor(*m, &pose, LB, LYAW))
            .collect();
        packets.push(lidar(1003, late_t, "A", local));
    }

    // Leap-second neighbourhood packet: a tiny backward SOW with the flag on
    // must be absorbed in the same generation (continuous clock clamped).
    let mut leap = gnss(seq_g + 1, T0 + 9.55, 0.0, 0.0, 0.0, "A");
    {
        let pose = pose_a(T0 + 9.55);
        leap.px = Some(pose.x);
        leap.py = Some(pose.y);
        leap.yaw = Some(pose.yaw);
    }
    leap.leap_second_flag = true;
    leap.timestamp = T0 + 9.55 - 0.5;
    packets.push(leap);

    // ---- GPS week rollover inside generation A ----------------------------
    // Replace one late lidar timestamp with a wrapped SOW (T0+8.75 is close
    // enough that adding WEEK_SECONDS demonstrates unwrapping).  We append it
    // as a dedicated tiny packet with explicit week handling in tests; here we
    // keep the demo timeline simple and instead include the rollover in the
    // test-suite.

    // ---- Generation B: reboot after T0+12 with sequence restart -----------
    let tb0 = T0 + 12.0;
    let mut seq_gb = 0;
    let mut seq_lb = 0;
    for i in 0..8 {
        let t = tb0 + i as f64 * 0.5;
        seq_gb += 1;
        let p = Pose {
            x: 6.0 + i as f64 * 0.4,
            y: 0.5,
            yaw: 0.4,
        };
        packets.push(gnss(seq_gb, t, p.x, p.y, p.yaw, "B"));
    }
    for i in 0..6 {
        let t = tb0 + 0.25 + i as f64 * 0.5;
        seq_lb += 1;
        let pose = Pose {
            x: 6.0 + (i as f64 + 0.5) * 0.4,
            y: 0.5,
            yaw: 0.4,
        };
        let local = lm
            .iter()
            .enumerate()
            .filter(|(k, _)| (k + i) % 5 == 0)
            .map(|(_, m)| map_to_sensor(*m, &pose, LB, LYAW))
            .collect();
        packets.push(lidar(seq_lb, t, "B", local));
    }

    // Reorder into receipt order: normal packets arrive time-ordered; the
    // duplicate of seq 1003 arrives late (after ~t=T0+2.25); generation B
    // arrives after the reboot gap.
    let mut late_dup: Vec<PacketInput> = Vec::new();
    let mut normal: Vec<PacketInput> = Vec::new();
    for pkt in packets.drain(..) {
        let is_dup = pkt.boot_marker.as_deref() == Some("A")
            && pkt.kind == "lidar"
            && pkt.seq == 1003;
        let seen = normal.iter().any(|q| {
            q.kind == pkt.kind && q.seq == pkt.seq && q.timestamp == pkt.timestamp
        });
        if is_dup && seen {
            late_dup.push(pkt);
        } else {
            normal.push(pkt);
        }
    }
    // Stable sort by boot generation then time, preserving the A block then B.
    normal.sort_by(|a, b| {
        a.boot_marker
            .cmp(&b.boot_marker)
            .then(a.timestamp.partial_cmp(&b.timestamp).unwrap())
    });
    // Inject the late duplicate just before the first packet after T0+2.0.
    let pos = normal
        .iter()
        .position(|q| q.timestamp > T0 + 2.0 && q.boot_marker.as_deref() == Some("A"))
        .unwrap_or(normal.len());
    for (i, d) in late_dup.into_iter().enumerate() {
        normal.insert(pos + i, d);
    }

    service::import_packets(c, normal).expect("demo import");

    // ---- Edges: calibration v1 valid for generation A --------------------
    let sensor = format!("lidar/{DEV}");
    let body = format!("body/{DEV}");
    let mapb = T0;
    let mape = T0 + 10.5;
    service::add_edge(
        c,
        edge(
            &sensor,
            &body,
            LB,
            LYAW,
            [1e-6, 1e-6, 1e-6],
            [1e-8, 1e-8, 1e-8],
            Some(mapb),
            Some(mape),
        ),
    )
    .expect("calib edge");

    // A *manual* body->map edge with a validity window entirely in the gap
    // region: creates a second, comparable (but lower precision) candidate
    // path for points that overlap its window.  It is deliberately loose.
    let _ = service::add_edge(
        c,
        edge(
            &body,
            "map",
            [0.0, 0.0, 0.0],
            0.0,
            [2e-2, 2e-2, 2e-2],
            [1e-4, 1e-4, 1e-4],
            Some(T0 + 6.0),
            Some(T0 + 6.3),
        ),
    );

    // Generation B calibration: same frames, new version (revision).
    service::add_edge(
        c,
        edge(
            &sensor,
            &body,
            [0.14, 0.01, -0.05],
            0.025,
            [8e-7, 8e-7, 8e-7],
            [8e-9, 8e-9, 8e-9],
            Some(T0 + 11.0),
            None,
        ),
    )
    .expect("calib edge v2");

    let _ = (tb0, seq_gb, seq_lb, map_to_sensor);
    let _ = c.query_row(
        "SELECT COUNT(*) FROM generations",
        [],
        |_r| Ok(()),
    );
    service::build_all(c).expect("build");
}
