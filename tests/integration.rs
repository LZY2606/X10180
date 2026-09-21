//! 端到端验收：时间环绕、重复序号、路径并列、奇异协方差、单位错误、
//! 局部失效、崩溃恢复。几何比较一律使用显式容差。

use pointcloud_jingwei::derive;
use pointcloud_jingwei::epoch::{EpochSplitter, GPS_WEEK_SECS};
use pointcloud_jingwei::graph::{Edge, Graph};
use pointcloud_jingwei::math::{Mat6, Quat, SE3, Vec3};
use pointcloud_jingwei::store::Store;

const EPS: f64 = 1e-9;

fn id_edge(src: &str, dst: &str, version: i64) -> Edge {
    Edge {
        src: src.into(),
        dst: dst.into(),
        version,
        tf: SE3::identity(),
        cov: Mat6::from_diag([0.01; 6]),
        valid_from: f64::NEG_INFINITY,
        valid_to: f64::INFINITY,
    }
}

/// 构造含一个代次、两个姿态、一个 LiDAR 包与标定的库。
fn seeded_store() -> Store {
    let mut s = Store::open(":memory:").unwrap();
    s.ingest("imu", "pose", 1, 0.0, "body", "rad", "姿态 t=0",
             serde_json::json!({"rot": Quat::identity(), "trans": Vec3::new(0.0,0.0,0.0)})).unwrap();
    s.ingest("imu", "pose", 2, 0.1, "body", "rad", "姿态 t=0.1",
             serde_json::json!({"rot": Quat::identity(), "trans": Vec3::new(1.0,0.0,0.0)})).unwrap();
    s.ingest("lidar", "lidar", 1, 0.05, "lidar", "m", "100 点扫描",
             serde_json::json!({"points": [Vec3::new(1.0, 0.0, 0.0)]})).unwrap();
    s.add_transform("lidar", "body",
        SE3::new(Quat::identity(), Vec3::new(0.5, 0.0, 0.0)),
        Mat6::from_diag([0.001; 6]), f64::NEG_INFINITY, f64::INFINITY).unwrap();
    s
}

#[test]
fn time_wraparound_week_rollover_single_epoch() {
    let mut s = Store::open(":memory:").unwrap();
    let a = s.ingest("d", "pose", 1, GPS_WEEK_SECS - 1.0, "body", "rad", "a", serde_json::json!({})).unwrap();
    let b = s.ingest("d", "pose", 2, 1.0, "body", "rad", "b", serde_json::json!({})).unwrap();
    assert_eq!(a.epoch, b.epoch, "GPS 周翻转不应划代");
    assert!((b.t - a.t - 2.0).abs() < EPS);
}

#[test]
fn duplicate_seq_after_restart_splits_epoch() {
    let mut s = Store::open(":memory:").unwrap();
    s.ingest("d", "pose", 5, 10.0, "body", "rad", "x", serde_json::json!({})).unwrap();
    let dup = s.ingest("d", "pose", 5, 10.0, "body", "rad", "重启后重复序号", serde_json::json!({})).unwrap();
    assert_eq!(dup.epoch, 1);
}

#[test]
fn leap_second_keeps_epoch_and_monotonic() {
    let mut sp = EpochSplitter::new();
    let a = sp.push(1, 500.0);
    let b = sp.push(2, 499.0); // 闰秒回退
    assert_eq!(a.epoch, b.epoch);
    assert!(b.t >= a.t);
}

#[test]
fn path_tie_deterministic_and_candidates_kept() {
    let mut g = Graph::default();
    g.add_edge(id_edge("lidar", "body", 1));
    g.add_edge(id_edge("body", "map", 1));
    let mut direct = id_edge("lidar", "map", 1);
    direct.cov = Mat6::from_diag([1.0; 6]); // 精度差
    g.add_edge(direct);
    let p1 = g.find_paths("lidar", "map");
    let p2 = g.find_paths("lidar", "map");
    assert_eq!(p1[0].frames, p2[0].frames, "选路必须确定");
    assert_eq!(p1[0].frames, vec!["lidar", "body", "map"]);
    assert_eq!(p1.len(), 2, "未选路径保留为候选");
}

#[test]
fn singular_covariance_accepted_indefinite_rejected() {
    let mut s = Store::open(":memory:").unwrap();
    // 奇异（零特征值）允许
    assert!(s.add_transform("a", "b", SE3::identity(), Mat6::from_diag([0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
        f64::NEG_INFINITY, f64::INFINITY).is_ok());
    // 负特征值拒绝
    assert!(s.add_transform("b", "c", SE3::identity(), Mat6::from_diag([0.0, -0.5, 0.0, 1.0, 1.0, 1.0]),
        f64::NEG_INFINITY, f64::INFINITY).is_err());
}

#[test]
fn unknown_unit_rejected_mm_converted() {
    let mut s = Store::open(":memory:").unwrap();
    assert!(s.ingest("d", "lidar", 1, 0.0, "lidar", "furlong", "坏单位", serde_json::json!({})).is_err());
    let mut s2 = seeded_store();
    s2.ingest("lidar", "lidar", 2, 0.05, "lidar", "mm", "毫米包",
              serde_json::json!({"points": [Vec3::new(1000.0, 0.0, 0.0)]})).unwrap();
    let st = derive::compute_missing(&mut s2, "lidar", "body", "map").unwrap();
    assert_eq!(st.failed, 0);
    let blocks = s2.blocks().unwrap();
    let mm_block = blocks.iter().find(|b| b.packet_id == 4).expect("mm 包应有块");
    // t=0.05 时 body 在 x=0.5，标定 +0.5，点 1000mm=1m → x = 0.5+0.5+1.0 = 2.0
    assert!((mm_block.points[0][0] - 2.0).abs() < 1e-6, "x={}", mm_block.points[0][0]);
}

#[test]
fn derived_point_matches_manual_chain_with_tolerance() {
    let mut s = seeded_store();
    let st = derive::compute_missing(&mut s, "lidar", "body", "map").unwrap();
    assert_eq!(st.computed, 1);
    let b = &s.blocks().unwrap()[0];
    // 姿态插值 t=0.05 → trans=(0.5,0,0)；标定平移 0.5；点 (1,0,0) → x=2.0
    assert!((b.points[0][0] - 2.0).abs() < EPS);
    assert!(b.points[0][1].abs() < EPS && b.points[0][2].abs() < EPS);
    // 来源链可反查：标定版本与姿态来源
    let hops = b.chain["hops"].as_array().unwrap();
    assert_eq!(hops[0]["edge"], "lidar->body");
    assert_eq!(hops[0]["version"], 1);
    assert_eq!(hops[1]["source"], "gnss-pose-interp");
}

#[test]
fn interp_never_crosses_epoch_or_large_gap() {
    use pointcloud_jingwei::derive::{interp_pose, PoseSample, MAX_POSE_INTERVAL};
    let poses = vec![
        PoseSample { epoch: 0, t: 0.0, pose: SE3::identity() },
        PoseSample { epoch: 0, t: 0.1, pose: SE3::identity() },
        PoseSample { epoch: 1, t: 10.0, pose: SE3::identity() },
        PoseSample { epoch: 1, t: 10.1, pose: SE3::identity() },
    ];
    assert!(interp_pose(&poses, 0, 0.05).is_some());
    assert!(interp_pose(&poses, 0, 10.05).is_none(), "不得跨代次插值");
    assert!(interp_pose(&poses, 0, 5.0).is_none(), "超出代次范围不外推");
    let sparse = vec![
        PoseSample { epoch: 0, t: 0.0, pose: SE3::identity() },
        PoseSample { epoch: 0, t: MAX_POSE_INTERVAL + 1.0, pose: SE3::identity() },
    ];
    assert!(interp_pose(&sparse, 0, 0.3).is_none(), "间隔超限不得插值");
}

#[test]
fn calibration_revision_invalidates_only_covered_window() {
    let mut s = seeded_store();
    // 第二个 LiDAR 包在不同时刻
    s.ingest("lidar", "lidar", 2, 0.08, "lidar", "m", "第二包",
             serde_json::json!({"points": [Vec3::new(2.0, 0.0, 0.0)]})).unwrap();
    derive::compute_missing(&mut s, "lidar", "body", "map").unwrap();
    assert_eq!(s.blocks().unwrap().len(), 2);
    // 标定修订：只覆盖 t∈[0.07, 0.09]
    s.add_transform("lidar", "body",
        SE3::new(Quat::identity(), Vec3::new(0.6, 0.0, 0.0)),
        Mat6::from_diag([0.001; 6]), 0.07, 0.09).unwrap();
    let n = s.invalidate_blocks("lidar", "body", 0.07, 0.09).unwrap();
    assert_eq!(n, 1, "只有覆盖时间段内的块失效");
    let remaining = s.blocks().unwrap();
    assert_eq!(remaining.len(), 1);
    assert!((remaining[0].t - 0.05).abs() < EPS, "窗口外的块保留");
    // 重算只补失效块
    let st = derive::compute_missing(&mut s, "lidar", "body", "map").unwrap();
    assert_eq!(st.computed, 1);
    assert_eq!(st.skipped_complete, 1);
    // 新块使用修订后的标定（平移 0.6）：x = 0.8 + 0.6 + 2.0 = 3.4
    let b2 = s.blocks().unwrap().into_iter().find(|b| (b.t - 0.08).abs() < EPS).unwrap();
    assert!((b2.points[0][0] - 3.4).abs() < EPS, "x={}", b2.points[0][0]);
    assert_eq!(b2.chain["hops"][0]["version"], 2);
}

#[test]
fn crash_recovery_drops_half_blocks_and_resumes() {
    let mut s = seeded_store();
    derive::compute_missing(&mut s, "lidar", "body", "map").unwrap();
    assert_eq!(s.blocks().unwrap().len(), 1);
    // 模拟崩溃：手工留下 pending 半块
    s.conn.execute(
        "INSERT INTO blocks(packet_id,status,chain,points,t) VALUES(999,'pending','[]','[]',0.05)",
        []).unwrap();
    // 重启（重新 open 会触发 recover；这里直接调用）
    let cleaned = s.recover().unwrap();
    assert_eq!(cleaned, 1, "pending 半块必须清除");
    assert_eq!(s.blocks().unwrap().len(), 1, "完整块不受影响");
    // 续算：完整块不重算
    let st = derive::compute_missing(&mut s, "lidar", "body", "map").unwrap();
    assert_eq!(st.computed, 0);
    assert_eq!(st.skipped_complete, 1);
}

#[test]
fn cycle_rejected_via_store_graph_rule() {
    let mut g = Graph::default();
    g.add_edge(id_edge("sensor", "body", 1));
    g.add_edge(id_edge("body", "map", 1));
    let closing = id_edge("map", "sensor", 1);
    let err = g.check_cycle(&closing).unwrap_err();
    assert_eq!(err.cycle, vec!["sensor", "body", "map", "sensor"]);
    assert!(err.residual.abs() < EPS, "恒等环残差应为 0");
}
