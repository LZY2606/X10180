//! 派生管线：把同一采集代次内的 LiDAR 包，经 姿态插值(body->map) 与
//! 标定变换(sensor->body) 投影到 map 帧，结果只作为可失效、可续算的缓存块。
//! 每个点的来源链（变换版本 + 误差传播）随块保存，可逐点反查。

use crate::graph::{Edge, Graph};
use crate::math::{Mat6, Quat, SE3, Vec3};
use crate::store::{unit_to_meters, Packet, Store, StoreError};

/// 姿态插值允许的最大间隔（秒）。
pub const MAX_POSE_INTERVAL: f64 = 0.5;

#[derive(Clone, Debug)]
pub struct PoseSample {
    pub epoch: u32,
    pub t: f64,
    pub pose: SE3, // body -> map
}

/// 同代次内线性插值；跨代次或间隔超限返回 None。
pub fn interp_pose(poses: &[PoseSample], epoch: u32, t: f64) -> Option<SE3> {
    let same: Vec<&PoseSample> = poses.iter().filter(|p| p.epoch == epoch).collect();
    if same.len() < 2 {
        return None;
    }
    let mut before: Option<&PoseSample> = None;
    let mut after: Option<&PoseSample> = None;
    for p in &same {
        if p.t <= t && before.map_or(true, |b| p.t >= b.t) {
            before = Some(p);
        }
        if p.t >= t && after.map_or(true, |a| p.t <= a.t) {
            after = Some(p);
        }
    }
    let (b, a) = (before?, after?);
    if t < same.first()?.t || t > same.last()?.t {
        return None; // 不外推
    }
    if a.t - b.t > MAX_POSE_INTERVAL {
        return None; // 超出允许间隔
    }
    if (a.t - b.t).abs() < 1e-12 {
        return Some(b.pose);
    }
    let f = (t - b.t) / (a.t - b.t);
    Some(SE3 {
        rot: b.pose.rot.slerp(a.pose.rot, f),
        trans: b.pose.trans.add(a.pose.trans.sub(b.pose.trans).scale(f)),
    })
}

/// 在有效时间内为 (src,dst) 选变换版本：覆盖 t 的版本中版本号最大者。
pub fn edge_at<'a>(edges: &'a [Edge], src: &str, dst: &str, t: f64) -> Option<&'a Edge> {
    edges
        .iter()
        .filter(|e| e.src == src && e.dst == dst && t >= e.valid_from && t < e.valid_to)
        .max_by_key(|e| e.version)
}

#[derive(Debug, serde::Serialize)]
pub struct DeriveStats {
    pub computed: usize,
    pub skipped_complete: usize,
    pub failed: usize,
}

/// 计算缺失的派生块；已 complete 的块原样保留（中断后可从完整块继续）。
pub fn compute_missing(store: &mut Store, sensor_frame: &str, body_frame: &str, map_frame: &str) -> Result<DeriveStats, StoreError> {
    let done = store.complete_block_packet_ids()?;
    let edges = store.transforms()?;
    let packets = store.packets(None)?;
    let poses: Vec<PoseSample> = packets
        .iter()
        .filter(|p| p.kind == "pose")
        .filter_map(|p| {
            let rot: Quat = serde_json::from_value(p.payload.get("rot")?.clone()).ok()?;
            let trans: Vec3 = serde_json::from_value(p.payload.get("trans")?.clone()).ok()?;
            Some(PoseSample { epoch: p.epoch, t: p.t, pose: SE3::new(rot, trans) })
        })
        .collect();

    let mut stats = DeriveStats { computed: 0, skipped_complete: 0, failed: 0 };
    for p in packets.iter().filter(|p| p.kind == "lidar") {
        if done.contains(&p.id) {
            stats.skipped_complete += 1;
            continue;
        }
        match compute_one(p, &poses, &edges, sensor_frame, body_frame, map_frame) {
            Ok((chain, points)) => {
                store.write_block(p.id, p.t, &chain, &points)?;
                stats.computed += 1;
            }
            Err(_) => stats.failed += 1,
        }
    }
    Ok(stats)
}

fn compute_one(
    p: &Packet,
    poses: &[PoseSample],
    edges: &[Edge],
    sensor_frame: &str,
    body_frame: &str,
    map_frame: &str,
) -> Result<(serde_json::Value, Vec<[f64; 3]>), StoreError> {
    let scale = unit_to_meters(&p.unit)?;
    let pose = interp_pose(poses, p.epoch, p.t)
        .ok_or_else(|| StoreError::BadInput(format!("包 {} 无可用姿态（跨代次或间隔超限）", p.id)))?;

    // 用变换图在 sensor->map 上选路；body->map 由姿态提供，sensor->body 由标定提供。
    let mut g = Graph::default();
    for e in edges {
        g.add_edge(e.clone());
    }
    let calib = edge_at(edges, sensor_frame, body_frame, p.t)
        .ok_or_else(|| StoreError::BadInput(format!("包 {} 时刻无有效标定 {}->{}", p.id, sensor_frame, body_frame)))?;

    // 误差传播：Σ_map = Ad(pose) Σ_calib Ad(pose)ᵀ（姿态视为已给定的确定量）
    let cov_map = Mat6::propagate(&pose, &Mat6::zero(), &calib.cov);

    let chain = serde_json::json!({
        "packet_id": p.id,
        "epoch": p.epoch,
        "t": p.t,
        "hops": [
            {"edge": format!("{}->{}", sensor_frame, body_frame), "version": calib.version,
             "valid_from": calib.valid_from, "valid_to": calib.valid_to},
            {"edge": format!("{}->{}", body_frame, map_frame), "version": null,
             "source": "gnss-pose-interp", "epoch": p.epoch}
        ],
        "cov_trace_map": cov_map.trace(),
    });

    let pts_in: Vec<Vec3> = serde_json::from_value(p.payload.get("points").cloned().unwrap_or_default())
        .map_err(|e| StoreError::BadInput(format!("点载荷解析失败: {e}")))?;
    let full = pose.compose(calib.tf);
    let out = pts_in
        .iter()
        .map(|q| {
            let m = full.apply(q.scale(scale));
            [m.x, m.y, m.z]
        })
        .collect();
    Ok((chain, out))
}
