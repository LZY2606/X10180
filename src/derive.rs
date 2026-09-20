//! 派生缓存：原始 LiDAR 包 -> map 系点云块。
//!
//! 一个数据包对应一个块。块状态机：
//! - 新块先以 `building` 落库；处理中断（进程崩溃）后留下的 building 块
//!   在下次构建时整体删除重来，因此“半块”永远不会被当成成功；
//! - 全部点与来源链写完后才在同一事务里置为 `ready`；
//! - 没有有效姿态或路径的块置为 `error` 并记录原因（跨代次/间隔过大等）。

use crate::geo::{point_cov, row_major, Se3};
use crate::graph::{resolve_path, EdgeRecord};
use crate::db::Db;
use anyhow::Result;
use nalgebra::Matrix6;
use rusqlite::params;
use serde::Serialize;

pub const MAX_POSE_GAP_SEC: f64 = 0.2;

#[derive(Debug, Clone, Serialize)]
pub struct BuildSummary {
    pub built: usize,
    pub skipped_ready: usize,
    pub recovered_stale: usize,
    pub errors: Vec<(i64, String)>,
}

struct PoseRow {
    t: f64,
    se3: Se3,
    cov: Matrix6<f64>,
}

/// 标定修订后使覆盖时间段内、且链上用到该 key 的 ready 块失效（局部失效）。
pub fn invalidate_for_edge(
    db: &Db,
    key: &str,
    valid_from: f64,
    valid_to: f64,
) -> Result<usize> {
    // 候选：点云时刻落在新版本有效窗内、且该块的来源链包含该 key。
    let affected: Vec<i64> = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let mut stmt = c.prepare(
            "SELECT DISTINCT b.id
             FROM block b
             JOIN packet p ON p.id=b.packet_id
             WHERE b.status='ready'
               AND p.unix_time>=?1 AND p.unix_time<?2
               AND EXISTS (
                 SELECT 1 FROM derived_point dp
                 JOIN point_chain_step s ON s.derived_point_id=dp.id
                 WHERE dp.block_id=b.id AND s.edge_key=?3)",
        )?;
        let rows = stmt
            .query_map(params![valid_from, valid_to, key], |r| r.get::<_, i64>(0))?
            .collect::<Result<Vec<_>, _>>()?;
        rows
    };
    let n = affected.len();
    {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        for id in &affected {
            c.execute("DELETE FROM block WHERE id=?1", params![id])?;
        }
        Db::log_event_locked(
            &c,
            "info",
            "blocks_invalidated",
            &format!("边 {key} 修订，{n} 个派生块局部失效"),
        );
    }
    Ok(n)
}

/// 选择 t 所属的同设备**姿态**代次（pose 与 lidar 按 kind 分代次）。
/// 先取时间窗包含 t 的代次；都不包含时选中心距 t 最近的代次（bracket 仍严格校验）。
fn pose_generation_for(db: &Db, device: &str, t: f64) -> Result<Option<i64>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    if let Some(id) = c
        .query_row(
            "SELECT g.id FROM generation g
             WHERE g.device_id=?1 AND g.kind='pose'
               AND g.t_start<=?2 AND g.t_end>=?2
             ORDER BY g.id ASC LIMIT 1",
            params![device, t],
            |r| r.get::<_, i64>(0),
        )
        .ok()
    {
        return Ok(Some(id));
    }
    Ok(c
        .query_row(
            "SELECT g.id FROM generation g
             WHERE g.device_id=?1 AND g.kind='pose' AND g.t_start IS NOT NULL
             ORDER BY (ABS((g.t_start+g.t_end)*0.5-?2)) ASC, g.id ASC LIMIT 1",
            params![device, t],
            |r| r.get::<_, i64>(0),
        )
        .ok())
}

/// 在同一姿态代次内取夹住 t 的相邻姿态；任何一边缺失即 None（绝不跨姿态代次）。
fn pose_brackets_in(
    db: &Db,
    device: &str,
    pose_gen: i64,
    t: f64,
) -> Result<Option<(PoseRow, PoseRow)>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let map_row = |r: &rusqlite::Row| -> rusqlite::Result<PoseRow> {
        let cov_json: String = r.get(8)?;
        let cov: Vec<f64> = serde_json::from_str(&cov_json).unwrap_or_default();
        let mut m = Matrix6::zeros();
        for i in 0..6 {
            for j in 0..6 {
                m[(i, j)] = cov.get(i * 6 + j).copied().unwrap_or(0.0);
            }
        }
        Ok(PoseRow {
            t: r.get(0)?,
            se3: Se3 {
                q: [r.get(1)?, r.get(2)?, r.get(3)?, r.get(4)?],
                t: [r.get(5)?, r.get(6)?, r.get(7)?],
            },
            cov: m,
        })
        // 列顺序：0=unix_time,1..4=q,5..7=t,8=cov_json
    };
    // 时间戳已规范化为全序连续刻度（闰秒 60.x 被放到次日 00:00 之后），严格取两侧。
    let before = c
        .query_row(
            "SELECT p.unix_time,s.qx,s.qy,s.qz,s.qw,s.tx,s.ty,s.tz,s.cov_json
             FROM packet p JOIN pose_sample s ON s.packet_id=p.id
             WHERE p.device_id=?1 AND p.kind='pose' AND p.gen_id=?2
                   AND p.is_duplicate=0 AND p.unix_time<=?3
             ORDER BY p.unix_time DESC, p.received_order DESC LIMIT 1",
            params![device, pose_gen, t],
            map_row,
        )
        .ok();
    let after = c
        .query_row(
            "SELECT p.unix_time,s.qx,s.qy,s.qz,s.qw,s.tx,s.ty,s.tz,s.cov_json
             FROM packet p JOIN pose_sample s ON s.packet_id=p.id
             WHERE p.device_id=?1 AND p.kind='pose' AND p.gen_id=?2
                   AND p.is_duplicate=0 AND p.unix_time>=?3
             ORDER BY p.unix_time ASC, p.received_order ASC LIMIT 1",
            params![device, pose_gen, t],
            map_row,
        )
        .ok();
Ok(match (before, after) {
        (Some(a), Some(b)) => Some((a, b)),
        _ => None,
    })
}

/// body@t -> map 的姿态虚拟边（id 为负值，不与持久边冲突）。
fn virtual_pose_edge(
    db: &Db,
    device: &str,
    _lidar_gen: i64,
    t: f64,
    serial: i64,
) -> Result<Option<(EdgeRecord, Se3, Matrix6<f64>, String)>> {
    let Some(pose_gen) = pose_generation_for(db, device, t)? else {
        return Ok(None);
    };
    let Some((a, b)) = pose_brackets_in(db, device, pose_gen, t)? else {
        return Ok(None);
    };
    let gap = b.t - a.t;
    let reason = if gap > MAX_POSE_GAP_SEC {
        format!("姿态时间缺口 {gap:.3}s 超过允许间隔 {MAX_POSE_GAP_SEC}s（不跨姿态代次插值）")
    } else {
        String::new()
    };
    let s = if (b.t - a.t).abs() < f64::EPSILON {
        0.0
    } else {
        (t - a.t) / (b.t - a.t)
    };
    let se3 = crate::geo::interpolate(&a.se3, &b.se3, s);
    let cov = crate::geo::interpolate_cov(&a.cov, &b.cov, s);
    Ok(Some((
        EdgeRecord {
            id: -(1000 + serial),
            key: format!("pose:{device}:g{pose_gen}"),
            version: 1,
            kind: "pose".into(),
            source_frame: format!("body:{device}"),
            target_frame: "map".into(),
            // body -> map（与点链 lidar -> body -> map 同向）
            q: se3.q,
            t: se3.t,
            cov: row_major(&cov),
            valid_from: t - 1.0,
            valid_to: t + 1.0,
            supersedes: None,
            active: true,
        },
        se3,
        cov,
        reason,
    )))
}


fn edge_fingerprint(db: &Db) -> Result<String> {
    let mut s = String::from("edges:");
    for e in crate::graph::list_edges(db, false)? {
        s.push_str(&format!(
            "{}={}[{:.3},{:.3}){};",
            e.key, e.version, e.valid_from, e.valid_to, e.cov.len()
        ));
    }
    Ok(s)
}

struct ChainEdge {
    record: EdgeRecord,
    se3: Se3,
    cov: Matrix6<f64>,
}

fn mark_error(
    db: &Db,
    packet_id: i64,
    gen_id: i64,
    fingerprint: &str,
    err: &str,
) -> Result<()> {
    let mut c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let tx = c.transaction()?;
    tx.execute("DELETE FROM block WHERE packet_id=?1", params![packet_id])?;
    tx.execute(
        "INSERT INTO block(packet_id,gen_id,status,fingerprint,error,built_at)
         VALUES(?1,?2,'error',?3,?4,NULL)",
        params![packet_id, gen_id, fingerprint, err],
    )?;
    tx.commit()?;
    Ok(())
}

fn ordered_chain(
    db: &Db,
    ids: &[i64],
    vpose: &EdgeRecord,
    vpose_cov: &Matrix6<f64>,
    at: f64,
) -> Result<Vec<ChainEdge>> {
    let mut out = Vec::new();
    for id in ids {
        if *id == vpose.id {
            out.push(ChainEdge {
                record: vpose.clone(),
                se3: vpose.se3(),
                cov: *vpose_cov,
            });
        } else {
            let e = crate::graph::list_edges(db, false)?
                .into_iter()
                .find(|e| e.id == *id && at >= e.valid_from && at < e.valid_to)
                .ok_or_else(|| anyhow::anyhow!("路径边 {id} 在时刻 {at} 失效"))?;
            let cov = e.cov6();
            let se3 = e.se3();
            out.push(ChainEdge { record: e, se3, cov });
        }
    }
    Ok(out)
}

fn build_one(db: &Db, packet_id: i64, fingerprint: &str) -> Result<Option<String>> {
    let (device, gen_id, t, unit): (String, i64, f64, String) = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        c.query_row(
            "SELECT device_id,gen_id,COALESCE(unix_time,0),unit FROM packet WHERE id=?1",
            params![packet_id],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
        )?
    };

    let (v_edge, _v_se3, v_cov, v_reason) = match virtual_pose_edge(db, &device, gen_id, t, packet_id)? {
        Some(v) => v,
        None => {
            let msg = "缺少同姿态代次内夹住该时刻的姿态对（不跨代次插值）".to_string();
            mark_error(db, packet_id, gen_id, fingerprint, &msg)?;
            return Ok(Some(msg));
        }
    };
    if !v_reason.is_empty() {
        mark_error(db, packet_id, gen_id, fingerprint, &v_reason)?;
        return Ok(Some(v_reason));
    }

    let lidar_frame = format!("lidar:{device}");
    let body_frame = format!("body:{device}");
    let choice = match resolve_path(db, &lidar_frame, "map", t, vec![v_edge.clone()]) {
        Ok(c) => c,
        Err(e) => {
            mark_error(db, packet_id, gen_id, fingerprint, &e.to_string())?;
            return Ok(Some(e.to_string()));
        }
    };
    if !choice.chosen.frames.iter().any(|f| f == &body_frame) {
        let msg = "选中路径未经过载体帧（缺少传感器到载体标定）".to_string();
        mark_error(db, packet_id, gen_id, fingerprint, &msg)?;
        return Ok(Some(msg));
    }

    let chain = match ordered_chain(db, &choice.chosen.edge_ids, &v_edge, &v_cov, t) {
        Ok(c) => c,
        Err(e) => {
            mark_error(db, packet_id, gen_id, fingerprint, &e.to_string())?;
            return Ok(Some(e.to_string()));
        }
    };

    let mut total = Se3::identity();
    let mut total_cov = Matrix6::identity() * 1e-15;
    for ce in &chain {
        let g = total.compose(&ce.se3);
        total_cov = crate::geo::compose_cov(&g, &total_cov, &ce.cov);
        total = g;
    }

    let scale = match crate::geo::length_to_meters(1.0, &unit) {
        Ok(s) => s,
        Err(e) => {
            mark_error(db, packet_id, gen_id, fingerprint, &e)?;
            return Ok(Some(e));
        }
    };

    let points: Vec<(i64, [f64; 3])> = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let mut stmt = c.prepare(
            "SELECT id,x,y,z FROM raw_point WHERE packet_id=?1 ORDER BY idx_in_packet",
        )?;
        let rows = stmt
            .query_map(params![packet_id], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    [r.get::<_, f64>(1)?, r.get::<_, f64>(2)?, r.get::<_, f64>(3)?],
                ))
            })?
            .collect::<Result<Vec<_>, _>>()?;
        rows
    };

    let mut c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let tx = c.transaction()?;
    tx.execute("DELETE FROM block WHERE packet_id=?1", params![packet_id])?;
    tx.execute(
        "INSERT INTO block(packet_id,gen_id,status,fingerprint,built_at)
         VALUES(?1,?2,'building',?3,NULL)",
        params![packet_id, gen_id, fingerprint],
    )?;
    let block_id = tx.last_insert_rowid();

    for (raw_id, raw_p) in points {
        let p_body = [raw_p[0] * scale, raw_p[1] * scale, raw_p[2] * scale];
        let map_p = total.transform_point(&p_body);
        let pc = point_cov(&total, &total_cov, &p_body);
        tx.execute(
            "INSERT INTO derived_point(block_id,raw_point_id,x,y,z,cov_xx,cov_yy,cov_zz,
                path_score,path_edge_ids)
             VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10)",
            params![
                block_id,
                raw_id,
                map_p[0], map_p[1], map_p[2],
                pc[(0, 0)], pc[(1, 1)], pc[(2, 2)],
                choice.chosen.cov_trace,
                serde_json::to_string(&choice.chosen.edge_ids)?,
            ],
        )?;
        let dp_id = tx.last_insert_rowid();
        for (step, ce) in chain.iter().enumerate() {
            tx.execute(
                "INSERT INTO point_chain_step(derived_point_id,step,edge_key,edge_version,
                    edge_kind,valid_from,valid_to,cov_trace)
                 VALUES(?1,?2,?3,?4,?5,?6,?7,?8)",
                params![
                    dp_id,
                    step as i64,
                    ce.record.key,
                    ce.record.version,
                    ce.record.kind,
                    ce.record.valid_from,
                    ce.record.valid_to,
                    crate::geo::precision_score(&ce.cov),
                ],
            )?;
        }
    }
    tx.execute(
        "UPDATE block SET status='ready', built_at=?1 WHERE id=?2",
        params![crate::db::now_unix(), block_id],
    )?;
    tx.commit()?;
    Ok(None)
}

/// 重建全部缺失块；返回前清理上次崩溃残留的 building/error 块。
pub fn build_all(db: &Db) -> Result<BuildSummary> {
    let stale = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let n: i64 = c.query_row(
            "SELECT COUNT(*) FROM block WHERE status!='ready'",
            [],
            |r| r.get(0),
        )?;
        n
    };
    {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        c.execute("DELETE FROM block WHERE status!='ready'", [])?;
    }

    let fingerprint = edge_fingerprint(db)?;
    let packets: Vec<i64> = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let mut stmt = c.prepare(
            "SELECT p.id FROM packet p
             WHERE p.kind='lidar' AND p.is_duplicate=0
               AND NOT EXISTS(SELECT 1 FROM block b WHERE b.packet_id=p.id AND b.status='ready')
             ORDER BY p.gen_id,p.unix_time,p.received_order",
        )?;
        let rows = stmt
            .query_map([], |r| r.get::<_, i64>(0))?
            .collect::<Result<Vec<_>, _>>()?;
        rows
    };

    let mut summary = BuildSummary {
        built: 0,
        skipped_ready: 0,
        recovered_stale: stale as usize,
        errors: Vec::new(),
    };
    for pid in packets {
        match build_one(db, pid, &fingerprint)? {
            None => summary.built += 1,
            Some(msg) => summary.errors.push((pid, msg)),
        }
    }
    Ok(summary)
}

/// 测试/运维辅助：制造一个 building 半块，用于验证崩溃恢复。
pub fn crash_mid_build(db: &Db) -> Result<()> {
    let pid: i64 = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        c.query_row(
            "SELECT id FROM packet WHERE kind='lidar' AND is_duplicate=0 ORDER BY id LIMIT 1",
            [],
            |r| r.get(0),
        )?
    };
    let fp = edge_fingerprint(db)?;
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    c.execute("DELETE FROM block WHERE packet_id=?1", params![pid])?;
    c.execute(
        "INSERT INTO block(packet_id,gen_id,status,fingerprint)
         SELECT id,gen_id,'building',?2 FROM packet WHERE id=?1",
        params![pid, fp],
    )?;
    Ok(())
}
