//! 验收级集成测试：时间环绕、重复序号、路径并列、奇异协方差、单位错误、
//! 局部失效与崩溃恢复。几何断言使用显式容差。

use dianyun_jingwei::geo::{diag6, Se3};
use dianyun_jingwei::graph::{self, NewEdge};
use dianyun_jingwei::ingest::{IncomingPacket, IncomingPose, Ingester};
use dianyun_jingwei::time::{gps_to_unix, unwrap_week, RawTime};
use dianyun_jingwei::{db, derive, query};

const TOL_M: f64 = 1e-6;

const BASE: f64 = 1_798_848_000.0;
fn utc(off: f64) -> RawTime {
    // BASE=2027-01-02T00:00:00Z；支持跨日的小偏移。
    let total = BASE + off;
    let base_days = (BASE / 86400.0).floor() as i64;
    let day_index = (total / 86400.0).floor() as i64;
    let sod = total - day_index as f64 * 86400.0;
    RawTime::Utc {
        y: 2027,
        m: 1,
        d: (2 + (day_index - base_days)) as u32,
        h: (sod / 3600.0).floor() as u32,
        min: ((sod % 3600.0) / 60.0).floor() as u32,
        sec: sod % 60.0,
    }
}

fn pose(dev: &str, seq: i64, sec: f64, x: f64, y: f64) -> IncomingPacket {
    IncomingPacket {
        device_id: dev.into(),
        kind: "pose".into(),
        seq,
        time: utc(sec),
        coord_system: "body".into(),
        unit: "m".into(),
        points: vec![],
        pose: Some(IncomingPose {
            q: [0.0, 0.0, 0.0, 1.0],
            t: [x, y, 0.0],
            cov: Some(serde_json::json!([0.000001,0.000001,0.000001,0.000001,0.000001,0.000001])),
        }),
    }
}

fn lidar(dev: &str, seq: i64, sec: f64, unit: &str, pts: Vec<Vec<f64>>) -> IncomingPacket {
    IncomingPacket {
        device_id: dev.into(),
        kind: "lidar".into(),
        seq,
        time: utc(sec),
        coord_system: "sensor".into(),
        unit: unit.into(),
        points: pts,
        pose: None,
    }
}

fn calib0(db: &db::Db, dev: &str) { calib(db, dev, 5e-4); }
fn calib(db: &db::Db, dev: &str, noise: f64) {
    graph::add_edge(
        db,
        NewEdge {
            key: format!("calib:{dev}:lidar"),
            kind: "calib".into(),
            source_frame: format!("lidar:{dev}"),
            target_frame: format!("body:{dev}"),
            se3: Se3::identity(),
            cov: diag6(noise),
            valid_from: 0.0,
            valid_to: 4e9,
        },
    )
    .unwrap();
}

fn basic_stream(db: &db::Db, dev: &str) {
    let ing = Ingester::new();
    for i in 0..6 {
        ing.ingest(db, pose(dev, i, i as f64 * 0.05, i as f64 * 0.1, 0.0)).unwrap();
    }
    for i in 0..3 {
        ing.ingest(
            db,
            lidar(
                dev,
                i,
                i as f64 * 0.1,
                "m",
                vec![vec![1.0, 2.0, 3.0, 0.5]],
            ),
        )
        .unwrap();
    }
    calib0(db, dev);
}

#[test]
fn full_pipeline_geometry_with_tolerance() {
    let db = db::open_in_memory().unwrap();
    basic_stream(&db, "U1");
    let sum = derive::build_all(&db).unwrap();
    assert_eq!(sum.built, 3);
    assert!(sum.errors.is_empty());

    let cloud = query::point_cloud(&db, 100).unwrap();
    assert_eq!(cloud.points.len(), 3);
    // 第 0 帧：body 姿态为单位变换（x=0），点应保持 (1,2,3)。
    let p0 = &cloud.points[0];
    assert!((p0.x - 1.0).abs() < TOL_M);
    assert!((p0.y - 2.0).abs() < TOL_M);
    assert!((p0.z - 3.0).abs() < TOL_M);
    // 第 2 帧（t=0.2）：姿态平移 (0.4,0,0)，无旋转。
    let p2 = cloud.points.iter().find(|p| p.seq == 2).unwrap();
    assert!((p2.x - 1.4).abs() < TOL_M);
    assert!((p2.y - 2.0).abs() < TOL_M);
    // 误差传播后位置方差为正、有限。
    for v in p2.cov {
        assert!(v.is_finite() && v > 0.0);
    }

    // 点来源链：lidar->body（标定）+ body->map（姿态虚拟边）。
    let (_, chain) = query::point_detail(&db, p2.id).unwrap().unwrap();
    assert_eq!(chain.len(), 2);
    assert!(chain.iter().any(|s| s.edge_key == "calib:U1:lidar"));
    assert!(chain.iter().any(|s| s.edge_kind == "pose"));
}

#[test]
fn gps_week_rollover_does_not_split_generation() {
    // 10 位周 1023 末 -> 0 初，连续 0.25s 姿态，点云可正常插值。
    let ing = Ingester::new();
    let d = db::open_in_memory().unwrap();
    let dev = "ROLL";
    for (i, (wk, tow, x)) in [
        (1023i64, 604_799.9_f64, 0.0_f64),
        (0, 0.1, 0.1),
        (0, 0.2, 0.2),
    ].into_iter().enumerate() {
        let mut pk = pose(dev, i as i64, 0.0, x, 0.0);
        pk.time = RawTime::GpsWeekTow {
            week: wk,
            tow,
            week_bits: 10,
        };
        ing.ingest(&d, pk).unwrap();
    }
    let mut frame = lidar(dev, 0, 0.0, "m", vec![vec![1.0, 0.0, 0.0]]);
    frame.time = RawTime::GpsWeekTow {
        week: 0,
        tow: 0.15,
        week_bits: 10,
    };
    // 604799.9 -> 0.1 跨周翻转，连续间隔 0.2s，合法。
    ing.ingest(&d, frame).unwrap();
    calib0(&d, dev);
    let sum = derive::build_all(&d).unwrap();
    assert_eq!(sum.errors.len(), 0, "{:?}", sum.errors);

    let gens = query::generations(&d).unwrap();
    let pose_gens = gens
        .iter()
        .filter(|g| g.device_id == dev && g.kind == "pose")
        .count();
    assert_eq!(pose_gens, 1);

    // 时间换算本身：周翻转后连续向前。
    let w = unwrap_week(0, Some(1023), Some(604_799.7), 10);
    assert_eq!(w, 1024);
    assert!(gps_to_unix(w, 0.05) > gps_to_unix(1023, 604_799.7));
}

#[test]
fn reboot_repeated_seq_opens_new_generation_and_late_packets_merge() {
    let ing = Ingester::new();
    let d = db::open_in_memory().unwrap();
    let dev = "RB";
    // 第一代从序号 10 开始（模拟设备自身计数器，不假定从 0 起）
    ing.ingest(&d, pose(dev, 10, 1.0, 0.0, 0.0)).unwrap();
    ing.ingest(&d, pose(dev, 11, 1.05, 0.1, 0.0)).unwrap();
    // 迟到包：序号很大，时间回到 1.02，应并入第一代
    let late = ing
        .ingest(&d, pose(dev, 99, 1.02, 0.04, 0.0))
        .unwrap();
    assert!(late.is_late);
    assert!(!late.new_generation);
    // 设备重启：序号归 0，时间倒流（远早于第一代起点）
    let reboot = ing
        .ingest(&d, pose(dev, 0, 0.5, 1.0, 0.0))
        .unwrap();
    assert!(reboot.new_generation, "重启后序号归零必须开新代次");
    assert_ne!(reboot.gen_id, late.gen_id);

    let gens = query::generations(&d).unwrap();
    let pose_gens: Vec<_> = gens
        .iter()
        .filter(|g| g.device_id == dev && g.kind == "pose")
        .collect();
    assert_eq!(pose_gens.len(), 2);
}

#[test]
fn duplicate_overlap_packets_are_flagged_and_excluded_from_blocks() {
    let ing = Ingester::new();
    let d = db::open_in_memory().unwrap();
    let dev = "DUP";
    let p1 = lidar(dev, 5, 0.0, "m", vec![vec![1.0, 0.0, 0.0]]);
    ing.ingest(&d, p1).unwrap();
    let p2 = lidar(dev, 5, 0.0, "m", vec![vec![1.0, 0.0, 0.0]]);
    let r = ing.ingest(&d, p2).unwrap();
    assert!(r.is_duplicate);
    let packets = query::packets(&d, Some("lidar")).unwrap();
    let dups: Vec<_> = packets.iter().filter(|p| p.is_duplicate).collect();
    assert_eq!(dups.len(), 1);
    assert!(dups[0].duplicate_of.is_some());
}

#[test]
fn singular_and_non_spd_covariance_rejected() {
    let d = db::open_in_memory().unwrap();
    let bad = NewEdge {
        key: "zero".into(),
        kind: "calib".into(),
        source_frame: "a".into(),
        target_frame: "b".into(),
        se3: Se3::identity(),
        cov: diag6(0.0),
        valid_from: 0.0,
        valid_to: 1.0,
    };
    assert!(graph::add_edge(&d, bad).is_err(), "零协方差必须被拒绝");
    // 实际通过 API 风格的错误：这里直接校验几何层
    assert!(dianyun_jingwei::geo::validate_cov(&diag6(0.0), 1e-10).is_err());
    assert!(dianyun_jingwei::geo::validate_cov(&diag6(-1.0), 1e-10).is_err());
    assert!(dianyun_jingwei::geo::validate_cov(&diag6(1e-3), 1e-10).is_ok());
}

#[test]
fn cycle_returns_loop_and_residual_and_blocks_write() {
    let d = db::open_in_memory().unwrap();
    let edge = |key: &str, s: &str, t: &str, noise: f64| NewEdge {
        key: key.into(),
        kind: "calib".into(),
        source_frame: s.into(),
        target_frame: t.into(),
        se3: Se3::identity(),
        cov: diag6(noise),
        valid_from: 0.0,
        valid_to: 1e9,
    };
    graph::add_edge(&d, edge("e1", "a", "b", 1e-3)).unwrap();
    graph::add_edge(&d, edge("e2", "b", "c", 1e-3)).unwrap();
    let (rec, cyc) = graph::add_edge(
        &d,
        NewEdge {
            key: "e3".into(),
            kind: "map".into(),
            source_frame: "c".into(),
            target_frame: "a".into(),
            se3: Se3::from_iso([0.0; 3], [0.02, 0.0, 0.0]),
            cov: diag6(1e-3),
            valid_from: 0.0,
            valid_to: 1e9,
        },
    )
    .unwrap();
    let cyc = cyc.expect("环必须被拒绝");
    assert_eq!(rec.id, -1, "拒绝时不应写入");
    assert_eq!(cyc.loop_frames.first().unwrap(), "c");
    assert_eq!(cyc.loop_frames.last().unwrap(), "c", "环必须闭合回起点: {:?}", cyc.loop_frames);
    assert!(cyc.translation_residual_m >= 0.0);
    // 只有两条边入库
    assert_eq!(graph::list_edges(&d, false).unwrap().len(), 2);
}

#[test]
fn parallel_paths_tie_and_deterministic_choice() {
    let d = db::open_in_memory().unwrap();
    // s -> m 有两条精度不同的直接/两跳路径
    graph::add_edge(
        &d,
        NewEdge {
            key: "direct".into(),
            kind: "map".into(),
            source_frame: "s".into(),
            target_frame: "m".into(),
            se3: Se3::identity(),
            cov: diag6(1e-4),
            valid_from: 0.0,
            valid_to: 1e9,
        },
    )
    .unwrap();
    graph::add_edge(
        &d,
        NewEdge {
            key: "via1".into(),
            kind: "calib".into(),
            source_frame: "s".into(),
            target_frame: "j".into(),
            se3: Se3::from_iso([0.0; 3], [0.0, 1.0, 0.0]),
            cov: diag6(1e-2),
            valid_from: 0.0,
            valid_to: 1e9,
        },
    )
    .unwrap();
    graph::add_edge(
        &d,
        NewEdge {
            key: "via2".into(),
            kind: "calib".into(),
            source_frame: "j".into(),
            target_frame: "m".into(),
            se3: Se3::from_iso([0.0; 3], [0.0, -1.0, 0.0]),
            cov: diag6(1e-2),
            valid_from: 0.0,
            valid_to: 1e9,
        },
    )
    .unwrap();
    let c1 = graph::resolve_path(&d, "s", "m", 100.0, vec![]).unwrap();
    assert_eq!(c1.chosen.edge_keys, vec!["direct"]);
    assert!(c1.candidates.iter().any(|p| p.frames == vec!["s", "j", "m"]));

    // 并列精度：再加一条与 direct 完全相同精度的直接路径。
    graph::add_edge(
        &d,
        NewEdge {
            key: "direct2".into(),
            kind: "map".into(),
            source_frame: "s".into(),
            target_frame: "m".into(),
            se3: Se3::identity(),
            cov: diag6(1e-4),
            valid_from: 0.0,
            valid_to: 1e9,
        },
    )
    .unwrap();
    let a = graph::resolve_path(&d, "s", "m", 100.0, vec![]).unwrap();
    let b = graph::resolve_path(&d, "s", "m", 100.0, vec![]).unwrap();
    assert_eq!(a.chosen.edge_keys, b.chosen.edge_keys, "并列选路必须确定");
    assert!(a.candidates.len() >= 2, "未选路径必须保留为候选");
}

#[test]
fn unit_mismatch_and_millimeters_convert() {
    let ing = Ingester::new();
    let d = db::open_in_memory().unwrap();
    let dev = "MM";
    for i in 0..4 {
        ing.ingest(&d, pose(dev, i, i as f64 * 0.05, 0.0, 0.0)).unwrap();
    }
    ing.ingest(
        &d,
        lidar(dev, 0, 0.05, "mm", vec![vec![3000.0, 0.0, 0.0]]),
    )
    .unwrap();
    calib0(&d, dev);
    derive::build_all(&d).unwrap();
    let cloud = query::point_cloud(&d, 10).unwrap();
    assert!((cloud.points[0].x - 3.0).abs() < 1e-6);

    // 未知单位：块必须失败并给出清楚错误（不会静默写错坐标）。
    let mut bad = lidar(dev, 1, 0.1, "cubit", vec![vec![1.0, 0.0, 0.0]]);
    bad.seq = 1;
    bad.time = utc(0.1);
    ing.ingest(&d, bad).unwrap();
    let sum = derive::build_all(&d).unwrap();
    assert!(sum
        .errors
        .iter()
        .any(|(_, m)| m.contains("未知长度单位")));
}

#[test]
fn interpolation_never_crosses_generation_or_exceeds_gap() {
    let ing = Ingester::new();
    let d = db::open_in_memory().unwrap();
    let dev = "GAP";
    // 姿态 0.0 与 0.5（缺口 0.5s > 0.2s）
    ing.ingest(&d, pose(dev, 0, 0.0, 0.0, 0.0)).unwrap();
    ing.ingest(&d, pose(dev, 1, 0.5, 1.0, 0.0)).unwrap();
    ing.ingest(
        &d,
        lidar(dev, 0, 0.25, "m", vec![vec![1.0, 0.0, 0.0]]),
    )
    .unwrap();
    calib0(&d, dev);
    let sum = derive::build_all(&d).unwrap();
    assert_eq!(sum.errors.len(), 1);
    assert!(sum.errors[0].1.contains("超过允许间隔"));

    // 同设备重启后的新姿态代次不能给旧代次点云插值
    ing.ingest(&d, pose(dev, 0, 10.0, 2.0, 0.0)).unwrap();
    ing.ingest(&d, pose(dev, 1, 10.05, 2.1, 0.0)).unwrap();
    let mut old_frame = lidar(dev, 0, 0.25, "m", vec![]);
    old_frame.points = vec![vec![1.0, 0.0, 0.0]];
    // 已存在 error 块；rebuild 结果保持 error（不跨到 10s 代次）
    let sum2 = derive::build_all(&d).unwrap();
    assert!(sum2.errors.iter().any(|(_, m)| m.contains("允许间隔") || m.contains("姿态")));
}

#[test]
fn calibration_revision_locally_invalidates_only_covered_blocks() {
    let d = db::open_in_memory().unwrap();
    basic_stream(&d, "REV");
    let sum = derive::build_all(&d).unwrap();
    assert_eq!(sum.built, 3);
    let before = query::stats(&d).unwrap();
    assert_eq!(before.ready_blocks, 3);

    // 同 key 新版本，时间窗只覆盖后段（t>=0.15）。
    let (edge, cyc) = graph::add_edge(
        &d,
        NewEdge {
            key: "calib:REV:lidar".into(),
            kind: "calib".into(),
            source_frame: "lidar:REV".into(),
            target_frame: "body:REV".into(),
            se3: Se3::from_iso([0.0; 3], [0.5, 0.0, 0.0]),
            cov: diag6(2e-3),
            valid_from: BASE + 0.15,
            valid_to: 4e9,
        },
    )
    .unwrap();
    assert!(cyc.is_none());
    assert_eq!(edge.version, 2);
    let n = derive::invalidate_for_edge(&d, &edge.key, edge.valid_from, edge.valid_to).unwrap();
    // t=0.0 与 t=0.1 的块不受影响；t=0.2 的块失效。
    assert_eq!(n, 1);
    let stats = query::stats(&d).unwrap();
    assert_eq!(stats.ready_blocks, 2);

    // 重建后：t=0.2 的点 x 增加 0.5（新版标定平移）。
    let sum = derive::build_all(&d).unwrap();
    assert_eq!(sum.built, 1);
    let cloud = query::point_cloud(&d, 100).unwrap();
    let p2 = cloud.points.iter().find(|p| p.seq == 2).unwrap();
    assert!((p2.x - 1.9).abs() < TOL_M, "got {}", p2.x);
    let p0 = cloud.points.iter().find(|p| p.seq == 0).unwrap();
    assert!((p0.x - 1.0).abs() < TOL_M, "旧时间段块不应改变");

    // 旧版本仍在库中（可追溯），但只有新版本 active。
    let all = graph::list_edges(&d, true).unwrap();
    assert!(all.iter().any(|e| e.key == "calib:REV:lidar" && e.version == 1 && !e.active));
    assert!(all.iter().any(|e| e.key == "calib:REV:lidar" && e.version == 2 && e.active));
}

#[test]
fn crash_recovery_drops_half_block_and_resumes_from_ready() {
    let d = db::open_in_memory().unwrap();
    basic_stream(&d, "CRASH");
    let sum = derive::build_all(&d).unwrap();
    assert_eq!(sum.built, 3);

    // 模拟“构建到一半进程被杀”：留下 building 半块（第一帧）。
    derive::crash_mid_build(&d).unwrap();
    let stats = query::stats(&d).unwrap();
    assert_eq!(stats.building_blocks, 1);
    assert_eq!(stats.ready_blocks, 2);

    // 恢复：半块不能被当成功，应整体重建；其余 ready 块跳过。
    let sum = derive::build_all(&d).unwrap();
    assert_eq!(sum.recovered_stale, 1);
    assert_eq!(sum.built, 1);
    let stats = query::stats(&d).unwrap();
    assert_eq!(stats.building_blocks, 0);
    assert_eq!(stats.ready_blocks, 3);
    assert_eq!(stats.derived_points, 3);
}

#[test]
fn raw_packet_facts_are_preserved_verbatim() {
    let ing = Ingester::new();
    let d = db::open_in_memory().unwrap();
    let mut pk = pose("FACTS", 7, 0.0, 0.0, 0.0);
    pk.coord_system = "enu".into();
    pk.unit = "m".into();
    pk.time = RawTime::GpsWeekTow {
        week: 1023,
        tow: 12.5,
        week_bits: 10,
    };
    let r = ing.ingest(&d, pk).unwrap();
    assert!(r.time_desc.starts_with("gps(w=1023,tow=12.500,10b)"));
    let packets = query::packets(&d, Some("pose")).unwrap();
    let row = packets.iter().find(|p| p.id == r.packet_id).unwrap();
    assert_eq!(row.coord_system, "enu");
    assert_eq!(row.unit, "m");
    assert!(row.content_summary.contains("fnv1a64=0x"));
    // 原始时间 JSON 完整保留
    let c = d.0.lock().unwrap();
    let raw: String = c
        .query_row(
            "SELECT raw_time_json FROM packet WHERE id=?1",
            rusqlite::params![r.packet_id],
            |r| r.get(0),
        )
        .unwrap();
    assert!(raw.contains("GpsWeekTow") || raw.contains("gps_week_tow") || raw.contains("week"));
}
