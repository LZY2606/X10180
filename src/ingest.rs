//! 数据包导入：先划分采集代次，再识别迟到包与重叠包。
//!
//! 关键不变量：
//! - 永远不“只按时间戳排序”；代次由序号重置/倒流等设备语义判定；
//! - 原始时间分量、坐标系、单位、摘要原样落库，点坐标保留导入单位；
//! - 重叠包（同代次同序号）与迟到包分别标记，重叠包不参与派生。

use crate::time::{to_unix, RawTime};
use crate::db::Db;
use anyhow::{anyhow, Result};
use rusqlite::params;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Mutex;

/// 迟到判定：时间落在此前代次范围内（外带 1s 容忍）即视为迟到包。
pub const LATE_TOL_SEC: f64 = 1.0;
/// 重叠判定：同代次同序号且时间差不超过此值。
pub const OVERLAP_TOL_SEC: f64 = 0.05;

#[derive(Debug, Clone, Deserialize)]
pub struct IncomingPose {
    /// 四元数 [x,y,z,w]
    pub q: [f64; 4],
    /// 平移（米）
    pub t: [f64; 3],
    /// 6 对角或 36 行优先
    #[serde(default)]
    pub cov: Option<serde_json::Value>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct IncomingPacket {
    pub device_id: String,
    pub kind: String,
    pub seq: i64,
    pub time: RawTime,
    #[serde(default = "default_coord")]
    pub coord_system: String,
    #[serde(default = "default_unit")]
    pub unit: String,
    #[serde(default)]
    pub points: Vec<Vec<f64>>,
    pub pose: Option<IncomingPose>,
}

fn default_coord() -> String {
    "sensor".into()
}
fn default_unit() -> String {
    "m".into()
}

#[derive(Debug, Clone, Serialize)]
pub struct IngestReport {
    pub packet_id: i64,
    pub gen_id: i64,
    pub new_generation: bool,
    pub generation_note: String,
    pub is_late: bool,
    pub is_duplicate: bool,
    pub unix_time: f64,
    pub time_desc: String,
    pub content_summary: String,
    pub point_count: usize,
}

#[allow(dead_code)]
struct GenState {
    gen_id: i64,
    seq_start: i64,
    last_seq: i64,
    /// 已收到（按接收顺序）的最大连续时间。
    t_max: f64,
    last_t: f64,
    t_start: f64,
    t_end: f64,
    note: String,
}

#[derive(Default)]
pub struct Ingester {
    // 设备 -> (上一原始周, 上一连续周, 上一 tow)，按接收顺序维护
    week_state: Mutex<HashMap<String, (i64, i64, f64)>>,
}

impl Ingester {
    pub fn new() -> Self {
        Self::default()
    }

    fn fnv1a64(bytes: &[u8]) -> u64 {
        let mut h: u64 = 0xcbf29ce484222325;
        for b in bytes {
            h ^= *b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
        h
    }

    /// 规范化时间：GPS 做周展开，其余直接换算。
    /// 状态保存“上一原始周 + 上一连续周”，选择与连续周最接近的候选，
    /// 使第 0 周开头的连续若干包都能被正确展开。
    fn normalize(&self, device: &str, raw: &RawTime) -> f64 {
        match raw {
            RawTime::GpsWeekTow {
                week,
                tow,
                week_bits,
            } => {
                let (raw_week, raw_tow, raw_bits) = (*week, *tow, *week_bits);
                let mut st = self.week_state.lock().unwrap();
                let modulus = 1i64 << raw_bits;
                let w = if let Some((prev_raw, prev_unwrapped, _prev_tow)) = st.get(device).copied() {
                    let k0 = prev_unwrapped.div_euclid(modulus);
                    let mut best = raw_week + k0 * modulus;
                    for cand in [best - modulus, best, best + modulus] {
                        if (cand - prev_unwrapped).abs() < (best - prev_unwrapped).abs() {
                            best = cand;
                        }
                    }
                    if best < prev_unwrapped && raw_week <= prev_raw {
                        raw_week
                    } else {
                        best
                    }
                } else {
                    raw_week.max(0)
                };
                
                st.insert(device.to_string(), (raw_week, w, raw_tow));
                to_unix(raw, Some(w))
            }
            other => to_unix(other, None),
        }
    }

    fn load_latest_gen(&self, conn: &rusqlite::Connection, device: &str, kind: &str) -> Result<Option<GenState>> {
        let row = conn
            .query_row(
                "SELECT g.id,g.seq_start,g.t_start,g.t_end,COALESCE(g.seq_head,g.seq_start),g.note,
                    COALESCE((SELECT MAX(unix_time) FROM packet WHERE gen_id=g.id AND is_late=0),g.t_start)
                 FROM generation g
                 WHERE g.device_id=?1 AND g.kind=?2
                 ORDER BY g.id DESC LIMIT 1",
                params![device, kind],
                |r| {
                    Ok(GenState {
                        gen_id: r.get(0)?,
                        seq_start: r.get(1)?,
                        last_seq: r.get(4)?,
                        last_t: r.get::<_, Option<f64>>(3)?.unwrap_or(0.0),
                        t_start: r.get::<_, Option<f64>>(2)?.unwrap_or(0.0),
                        t_end: r.get::<_, Option<f64>>(3)?.unwrap_or(0.0),
                        note: r.get(5)?,
                        t_max: r.get::<_, Option<f64>>(6)?.unwrap_or(0.0),
                    })
                },
            )
            .ok();
        Ok(row)
    }

    fn find_late_gen(
        &self,
        conn: &rusqlite::Connection,
        device: &str,
        kind: &str,
        t: f64,
    ) -> Result<Option<i64>> {
        let id = conn
            .query_row(
                "SELECT g.id FROM generation g
                 WHERE g.device_id=?1 AND g.kind=?2
                   AND g.t_start-?3 <= ?4 AND g.t_end+?3 >= ?4
                 ORDER BY g.id DESC LIMIT 1",
                params![device, kind, LATE_TOL_SEC, t],
                |r| r.get::<_, i64>(0),
            )
            .ok();
        Ok(id)
    }

    /// 跨代次查找同设备同 kind 的重叠包：同序号且时间吻合。
    fn find_overlap(
        &self,
        conn: &rusqlite::Connection,
        device: &str,
        kind: &str,
        seq: i64,
        t: f64,
    ) -> Result<Option<(i64, i64)>> {
        let r = conn
            .query_row(
                "SELECT gen_id,id FROM packet
                 WHERE device_id=?1 AND kind=?2 AND seq=?3 AND is_duplicate=0
                   AND unix_time IS NOT NULL
                   AND ABS(unix_time-?4)<=?5
                 ORDER BY id LIMIT 1",
                params![device, kind, seq, t, OVERLAP_TOL_SEC],
                |r| Ok((r.get::<_, i64>(0)?, r.get::<_, i64>(1)?)),
            )
            .ok();
        Ok(r)
    }

    fn find_same_seq(
        &self,
        conn: &rusqlite::Connection,
        gen_id: i64,
        seq: i64,
        t: f64,
    ) -> Result<Option<(i64, f64, String)>> {
        let r = conn
            .query_row(
                "SELECT id,COALESCE(unix_time,0),content_summary FROM packet
                 WHERE gen_id=?1 AND seq=?2 AND is_duplicate=0
                 ORDER BY id LIMIT 1",
                params![gen_id, seq],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
            )
            .ok();
        // 仅在时间接近时才算重叠，避免重启后巧合的同序号。
        if let Some((_, pt, _)) = r {
            let pt: f64 = pt;
            if (pt - t).abs() <= OVERLAP_TOL_SEC || pt == 0.0 {
                return Ok(r);
            }
        }
        Ok(None)
    }

    pub fn ingest(&self, db: &Db, p: IncomingPacket) -> Result<IngestReport> {
        if p.kind != "lidar" && p.kind != "pose" {
            return Err(anyhow!("kind 必须是 lidar 或 pose"));
        }
        if p.kind == "lidar" {
            for pt in &p.points {
                if pt.len() < 3 {
                    return Err(anyhow!("点至少需要 x,y,z"));
                }
                pt.iter().take(3).try_for_each(|v| {
                    if v.is_finite() {
                        Ok(())
                    } else {
                        Err(anyhow!("点坐标含非有限值"))
                    }
                })?;
            }
        }
        if let Some(pose) = &p.pose {
            pose.q.iter().chain(pose.t.iter()).try_for_each(|v| {
                if v.is_finite() {
                    Ok(())
                } else {
                    Err(anyhow!("姿态含非有限值"))
                }
            })?;
        }

        let t = self.normalize(&p.device_id, &p.time);
        if !t.is_finite() {
            return Err(anyhow!("时间戳换算结果非有限"));
        }
        let time_desc = p.time.describe();
        let raw_time_json = serde_json::to_string(&p.time)?;
        let point_n = p.points.len();
        let summary_payload = serde_json::to_string(&(
            &p.seq,
            &p.time,
            &p.coord_system,
            &p.unit,
            point_n,
            &p.pose.as_ref().map(|po| (po.q, po.t)),
        ))?;
        let summary = format!(
            "n={};bytes={};fnv1a64=0x{:016x}",
            point_n,
            summary_payload.len(),
            Self::fnv1a64(summary_payload.as_bytes())
        );

        let mut c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let tx = c.transaction()?;

        let latest = self.load_latest_gen(&tx, &p.device_id, &p.kind)?;
        let (gen_id, new_generation, is_late, note) = match &latest {
            None => {
                let id = insert_gen(&tx, &p.device_id, &p.kind, p.seq, t, "首包")?;
                (id, true, false, "首包".to_string())
            }
            Some(g) => {
                // 1) 重叠/重传：同设备同 kind 任意既有代次中已有同序号且时间吻合。
                if let Some((dup_gen, dup_id)) = self.find_overlap(
                    &tx, &p.device_id, &p.kind, p.seq, t,
                )? {
                    let _ = (dup_id, summary.len());
                    (dup_gen, false, false, g.note.clone())
                } else {
                    // 2) 序号重置 = 设备重启/新序列；或序号回到代次起点且时间显著倒流。
                    let seq_reset = p.seq < g.last_seq;
                    let wraps_to_start =
                        p.seq == g.seq_start && t < g.t_start - LATE_TOL_SEC;
                    if seq_reset || wraps_to_start {
                        let reason = "序号重置（设备重启或周翻转后新序列）";
                        let id = insert_gen(&tx, &p.device_id, &p.kind, p.seq, t, reason)?;
                        (id, true, false, reason.to_string())
                    } else {
                        // 3) 迟到包：时间落回既有代次窗，且早于已收到的最大时间（乱序回补）。
                        let in_window = self
                            .find_late_gen(&tx, &p.device_id, &p.kind, t)?
                            .is_some();
                        if in_window && t < g.t_max - 1e-6 {
                            (g.gen_id, false, true, "迟到包归入既有代次".to_string())
                        } else {
                            (g.gen_id, false, false, g.note.clone())
                        }
                    }
                }
            }
        };

        // 重叠包：同代次同序号、时间吻合。
        let same = self.find_same_seq(&tx, gen_id, p.seq, t)?;
        let (is_dup, dup_of) = match same {
            Some((id, _pt, _prev_summary)) => (true, Some(id)),
            None => (false, None),
        };

        let order: i64 = tx.query_row(
            "SELECT COALESCE(MAX(received_order),0)+1 FROM packet",
            [],
            |r| r.get(0),
        )?;
        tx.execute(
            "INSERT INTO packet(device_id,kind,seq,gen_id,raw_time_json,time_desc,unix_time,
                coord_system,unit,content_summary,received_order,is_duplicate,is_late,duplicate_of,raw_blob_len)
             VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13,?14,?15)",
            params![
                p.device_id,
                p.kind,
                p.seq,
                gen_id,
                raw_time_json,
                time_desc,
                t,
                p.coord_system,
                p.unit,
                summary,
                order,
                if is_dup {1i64} else {0},
                if is_late {1i64} else {0},
                dup_of,
                summary_payload.len() as i64,
            ],
        )?;
        let packet_id = tx.last_insert_rowid();

        for (i, pt) in p.points.iter().enumerate() {
            let intensity = pt.get(3).copied().unwrap_or(0.0);
            tx.execute(
                "INSERT INTO raw_point(packet_id,idx_in_packet,x,y,z,intensity)
                 VALUES(?1,?2,?3,?4,?5,?6)",
                params![packet_id, i as i64, pt[0], pt[1], pt[2], intensity],
            )?;
        }

        if let Some(pose) = &p.pose {
            let cov = match &pose.cov {
                Some(v) => crate::geo::parse_cov(v).map_err(|e| anyhow!(e))?,
                None => crate::geo::diag6(1e-3),
            };
            crate::geo::validate_cov(&cov, 1e-12).map_err(|e| anyhow!(e))?;
            let cov_json = serde_json::to_string(&crate::geo::row_major(&cov))?;
            tx.execute(
                "INSERT INTO pose_sample(packet_id,qx,qy,qz,qw,tx,ty,tz,cov_json)
                 VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9)",
                params![
                    packet_id, pose.q[0], pose.q[1], pose.q[2], pose.q[3],
                    pose.t[0], pose.t[1], pose.t[2], cov_json
                ],
            )?;
        }

        if !is_late && !is_dup {
            // 只接受“紧接头”的主序列包推进头；乱序高序号由迟到路径处理，不污染头。
            tx.execute(
                "UPDATE generation
                 SET seq_head = CASE
                       WHEN seq_head IS NULL THEN ?2
                       WHEN ?2 = seq_head + 1 THEN ?2
                       ELSE seq_head END
                 WHERE id=?1",
                params![gen_id, p.seq],
            )?;
        }
        // 维护代次时间窗。
        tx.execute(
            "UPDATE generation SET
                t_start = (SELECT MIN(unix_time) FROM packet WHERE gen_id=?1 AND unix_time IS NOT NULL),
                t_end   = (SELECT MAX(unix_time) FROM packet WHERE gen_id=?1 AND unix_time IS NOT NULL)
             WHERE id=?1",
            params![gen_id],
        )?;
        tx.commit()?;
        drop(c);

        Ok(IngestReport {
            packet_id,
            gen_id,
            new_generation,
            generation_note: note,
            is_late,
            is_duplicate: is_dup,
            unix_time: t,
            time_desc,
            content_summary: summary,
            point_count: point_n,
        })
    }
}

fn insert_gen(
    tx: &rusqlite::Transaction,
    device: &str,
    kind: &str,
    seq: i64,
    t: f64,
    note: &str,
) -> Result<i64> {
    tx.execute(
        "INSERT INTO generation(device_id,kind,seq_start,seq_head,t_start,t_end,note)
         VALUES(?1,?2,?3,?3,?4,?4,?5)",
        params![device, kind, seq, t, note],
    )?;
    Ok(tx.last_insert_rowid())
}
