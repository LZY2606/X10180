//! 面向浏览器与验收的只读查询：帧链、时间缺口、点来源链、路径候选。

use crate::db::Db;
use anyhow::Result;
use rusqlite::params;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct GenerationDto {
    pub id: i64,
    pub device_id: String,
    pub kind: String,
    pub seq_start: i64,
    pub t_start: Option<f64>,
    pub t_end: Option<f64>,
    pub note: String,
    pub packet_count: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct PacketDto {
    pub id: i64,
    pub device_id: String,
    pub kind: String,
    pub seq: i64,
    pub gen_id: i64,
    pub time_desc: String,
    pub unix_time: Option<f64>,
    pub coord_system: String,
    pub unit: String,
    pub content_summary: String,
    pub received_order: i64,
    pub is_duplicate: bool,
    pub is_late: bool,
    pub duplicate_of: Option<i64>,
    pub point_count: i64,
    pub block_status: Option<String>,
    pub block_error: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct GapDto {
    pub device_id: String,
    pub gen_id: i64,
    pub from_packet: i64,
    pub to_packet: i64,
    pub from_time: f64,
    pub to_time: f64,
    pub gap_sec: f64,
    pub kind: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct DerivedPointDto {
    pub id: i64,
    pub raw_point_id: i64,
    pub block_id: i64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub cov: [f64; 3],
    pub path_score: f64,
    pub raw: [f64; 3],
    pub raw_unit: String,
    pub device_id: String,
    pub packet_id: i64,
    pub gen_id: i64,
    pub unix_time: Option<f64>,
    pub seq: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ChainStepDto {
    pub step: i64,
    pub edge_key: String,
    pub edge_version: i64,
    pub edge_kind: String,
    pub valid_from: f64,
    pub valid_to: f64,
    pub cov_trace: f64,
}

pub fn generations(db: &Db) -> Result<Vec<GenerationDto>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut stmt = c.prepare(
        "SELECT g.id,g.device_id,g.kind,g.seq_start,g.t_start,g.t_end,g.note,
                (SELECT COUNT(*) FROM packet p WHERE p.gen_id=g.id)
         FROM generation g ORDER BY g.id",
    )?;
    let rows = stmt
        .query_map([], |r| {
            Ok(GenerationDto {
                id: r.get(0)?,
                device_id: r.get(1)?,
                kind: r.get(2)?,
                seq_start: r.get(3)?,
                t_start: r.get(4)?,
                t_end: r.get(5)?,
                note: r.get(6)?,
                packet_count: r.get(7)?,
            })
        })?
        .collect::<Result<Vec<_>, _>>()?;
    Ok(rows)
}

pub fn packets(db: &Db, kind: Option<&str>) -> Result<Vec<PacketDto>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let sql = format!(
        "SELECT p.id,p.device_id,p.kind,p.seq,p.gen_id,p.time_desc,p.unix_time,
                p.coord_system,p.unit,p.content_summary,p.received_order,
                p.is_duplicate,p.is_late,p.duplicate_of,
                (SELECT COUNT(*) FROM raw_point r WHERE r.packet_id=p.id),
                b.status,COALESCE(b.error,'')
         FROM packet p LEFT JOIN block b ON b.packet_id=p.id
         WHERE (?1 IS NULL OR p.kind=?1)
         ORDER BY p.received_order"
    );
    let mut stmt = c.prepare(&sql)?;
    let rows = stmt
        .query_map(params![kind], |r| {
            Ok(PacketDto {
                id: r.get(0)?,
                device_id: r.get(1)?,
                kind: r.get(2)?,
                seq: r.get(3)?,
                gen_id: r.get(4)?,
                time_desc: r.get(5)?,
                unix_time: r.get(6)?,
                coord_system: r.get(7)?,
                unit: r.get(8)?,
                content_summary: r.get(9)?,
                received_order: r.get(10)?,
                is_duplicate: r.get::<_, i64>(11)? != 0,
                is_late: r.get::<_, i64>(12)? != 0,
                duplicate_of: r.get(13)?,
                point_count: r.get(14)?,
                block_status: r.get(15)?,
                block_error: r.get(16)?,
            })
        })?
        .collect::<Result<Vec<_>, _>>()?;
    Ok(rows)
}

/// 同设备同代次内相邻非重复包的时间缺口；不跨代次比较。
pub fn gaps(db: &Db, threshold_sec: f64) -> Result<Vec<GapDto>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut stmt = c.prepare(
        "SELECT device_id,kind,gen_id,id,unix_time,seq FROM packet
         WHERE is_duplicate=0 AND unix_time IS NOT NULL
         ORDER BY device_id,kind,gen_id,unix_time,received_order",
    )?;
    let rows = stmt
        .query_map([], |r| {
            Ok((
                r.get::<_, String>(0)?,
                r.get::<_, String>(1)?,
                r.get::<_, i64>(2)?,
                r.get::<_, i64>(3)?,
                r.get::<_, f64>(4)?,
                r.get::<_, i64>(5)?,
            ))
        })?
        .collect::<Result<Vec<_>, _>>()?;
    let mut out = Vec::new();
    for w in rows.windows(2) {
        let (dev, kind, gen, id0, t0, _) = &w[0];
        let (_, _, gen2, id1, t1, _) = &w[1];
        if gen != gen2 {
            continue;
        }
        let gap = t1 - t0;
        if gap > threshold_sec {
            out.push(GapDto {
                device_id: dev.clone(),
                gen_id: *gen,
                from_packet: *id0,
                to_packet: *id1,
                from_time: *t0,
                to_time: *t1,
                gap_sec: gap,
                kind: kind.clone(),
            });
        }
    }
    Ok(out)
}

#[derive(Debug, Clone, Serialize)]
pub struct PointCloudResp {
    pub points: Vec<DerivedPointDto>,
    pub gen_count: i64,
    pub block_count: i64,
}

/// 帧着色点云：每个 LiDAR 包（帧）一个颜色（由前端按 packet_id 着色）。
pub fn point_cloud(db: &Db, limit: i64) -> Result<PointCloudResp> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut stmt = c.prepare(
        "SELECT dp.id,dp.raw_point_id,dp.block_id,dp.x,dp.y,dp.z,
                dp.cov_xx,dp.cov_yy,dp.cov_zz,dp.path_score,
                rp.x,rp.y,rp.z,p.unit,p.device_id,p.id,p.gen_id,p.unix_time,p.seq
         FROM derived_point dp
         JOIN block b ON b.id=dp.block_id
         JOIN raw_point rp ON rp.id=dp.raw_point_id
         JOIN packet p ON p.id=b.packet_id
         WHERE b.status='ready'
         ORDER BY p.gen_id,p.unix_time,p.seq,dp.id
         LIMIT ?1",
    )?;
    let points = stmt
        .query_map(params![limit], |r| {
            Ok(DerivedPointDto {
                id: r.get(0)?,
                raw_point_id: r.get(1)?,
                block_id: r.get(2)?,
                x: r.get(3)?,
                y: r.get(4)?,
                z: r.get(5)?,
                cov: [r.get(6)?, r.get(7)?, r.get(8)?],
                path_score: r.get(9)?,
                raw: [r.get(10)?, r.get(11)?, r.get(12)?],
                raw_unit: r.get(13)?,
                device_id: r.get(14)?,
                packet_id: r.get(15)?,
                gen_id: r.get(16)?,
                unix_time: r.get(17)?,
                seq: r.get(18)?,
            })
        })?
        .collect::<Result<Vec<_>, _>>()?;
    let gen_count = c.query_row("SELECT COUNT(*) FROM generation", [], |r| r.get::<_, i64>(0))?;
    let block_count = c.query_row(
        "SELECT COUNT(*) FROM block WHERE status='ready'",
        [],
        |r| r.get::<_, i64>(0),
    )?;
    Ok(PointCloudResp {
        points,
        gen_count,
        block_count,
    })
}

pub fn point_detail(db: &Db, derived_id: i64) -> Result<Option<(DerivedPointDto, Vec<ChainStepDto>)>> {
    let pc = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        c.query_row(
            "SELECT dp.id,dp.raw_point_id,dp.block_id,dp.x,dp.y,dp.z,
                    dp.cov_xx,dp.cov_yy,dp.cov_zz,dp.path_score,
                    rp.x,rp.y,rp.z,p.unit,p.device_id,p.id,p.gen_id,p.unix_time,p.seq
             FROM derived_point dp
             JOIN block b ON b.id=dp.block_id
             JOIN raw_point rp ON rp.id=dp.raw_point_id
             JOIN packet p ON p.id=b.packet_id
             WHERE dp.id=?1",
            params![derived_id],
            |r| {
                Ok(DerivedPointDto {
                    id: r.get(0)?,
                    raw_point_id: r.get(1)?,
                    block_id: r.get(2)?,
                    x: r.get(3)?,
                    y: r.get(4)?,
                    z: r.get(5)?,
                    cov: [r.get(6)?, r.get(7)?, r.get(8)?],
                    path_score: r.get(9)?,
                    raw: [r.get(10)?, r.get(11)?, r.get(12)?],
                    raw_unit: r.get(13)?,
                    device_id: r.get(14)?,
                    packet_id: r.get(15)?,
                    gen_id: r.get(16)?,
                    unix_time: r.get(17)?,
                    seq: r.get(18)?,
                })
            },
        )
        .ok()
    };
    let Some(pc) = pc else { return Ok(None) };
    let chain = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let mut stmt = c.prepare(
            "SELECT step,edge_key,edge_version,edge_kind,valid_from,valid_to,cov_trace
             FROM point_chain_step WHERE derived_point_id=?1 ORDER BY step",
        )?;
        let rows = stmt
            .query_map(params![derived_id], |r| {
                Ok(ChainStepDto {
                    step: r.get(0)?,
                    edge_key: r.get(1)?,
                    edge_version: r.get(2)?,
                    edge_kind: r.get(3)?,
                    valid_from: r.get(4)?,
                    valid_to: r.get(5)?,
                    cov_trace: r.get(6)?,
                })
            })?
            .collect::<Result<Vec<_>, _>>()?;
        rows
    };
    Ok(Some((pc, chain)))
}

#[derive(Debug, Clone, Serialize)]
pub struct TrajectoryPoint {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub device_id: String,
    pub gen_id: i64,
}

pub fn trajectory(db: &Db) -> Result<Vec<TrajectoryPoint>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut stmt = c.prepare(
        "SELECT p.unix_time,s.tx,s.ty,s.tz,p.device_id,p.gen_id
         FROM packet p JOIN pose_sample s ON s.packet_id=p.id
         WHERE p.is_duplicate=0
         ORDER BY p.device_id,p.gen_id,p.unix_time",
    )?;
    let rows = stmt
        .query_map([], |r| {
            Ok(TrajectoryPoint {
                t: r.get(0)?,
                x: r.get(1)?,
                y: r.get(2)?,
                z: r.get(3)?,
                device_id: r.get(4)?,
                gen_id: r.get(5)?,
            })
        })?
        .collect::<Result<Vec<_>, _>>()?;
    Ok(rows)
}

#[derive(Debug, Clone, Serialize)]
pub struct Stats {
    pub packets: i64,
    pub duplicates: i64,
    pub late: i64,
    pub generations: i64,
    pub raw_points: i64,
    pub ready_blocks: i64,
    pub building_blocks: i64,
    pub error_blocks: i64,
    pub derived_points: i64,
    pub edges_active: i64,
    pub edges_versions: i64,
}

pub fn stats(db: &Db) -> Result<Stats> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let count = |sql: &str| -> rusqlite::Result<i64> {
        c.query_row(sql, [], |r| r.get(0))
    };
    Ok(Stats {
        packets: count("SELECT COUNT(*) FROM packet")?,
        duplicates: count("SELECT COUNT(*) FROM packet WHERE is_duplicate=1")?,
        late: count("SELECT COUNT(*) FROM packet WHERE is_late=1")?,
        generations: count("SELECT COUNT(*) FROM generation")?,
        raw_points: count("SELECT COUNT(*) FROM raw_point")?,
        ready_blocks: count("SELECT COUNT(*) FROM block WHERE status='ready'")?,
        building_blocks: count("SELECT COUNT(*) FROM block WHERE status='building'")?,
        error_blocks: count("SELECT COUNT(*) FROM block WHERE status='error'")?,
        derived_points: count("SELECT COUNT(*) FROM derived_point")?,
        edges_active: count("SELECT COUNT(*) FROM edge WHERE active=1")?,
        edges_versions: count("SELECT COUNT(*) FROM edge")?,
    })
}
