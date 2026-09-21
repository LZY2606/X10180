//! Application service layer: import, generation partitioning, pose
//! interpolation, transform routing and derived-block construction.

use crate::db::{recover_pending, ALGO_VERSION};
use crate::graph::{self, Edge, RouteArc};
use crate::math::{
    vadd, vscale, Cov6, Iso, Quat,
};
use crate::model::{
    AppError, AppResult, PacketInput, PacketKind,
};
use crate::time::{is_generation_break, ContinuousTime, RawTime, Unwrapper};
use crate::units::{AngleUnit, LengthUnit, UnitSpec};
use rusqlite::{params, Connection};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const BLOCK_SECS: f64 = 0.5;
/// Maximum allowed time gap for pose interpolation across a query time.
pub const MAX_INTERP_GAP: f64 = 0.8;

#[derive(Clone, Debug, Serialize)]
pub struct ImportReport {
    pub accepted: usize,
    pub duplicates: usize,
    pub rejected: usize,
    pub new_generations: Vec<i64>,
    pub errors: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct BuildReport {
    pub blocks_complete: usize,
    pub blocks_rebuilt: usize,
    pub blocks_skipped: usize,
    pub points_aligned: usize,
    pub points_unaligned: usize,
}

#[derive(Clone, Debug, Deserialize)]
pub struct EdgeInput {
    pub source: String,
    pub target: String,
    #[serde(default)]
    pub valid_from: Option<f64>,
    #[serde(default)]
    pub valid_to: Option<f64>,
    pub translation: [f64; 3],
    /// Quaternion [w,x,y,z] or euler [roll,pitch,yaw].
    #[serde(default)]
    pub quat: Option<[f64; 4]>,
    #[serde(default)]
    pub euler_rpy: Option<[f64; 3]>,
    pub covariance: Vec<f64>,
    #[serde(default)]
    pub origin: Option<String>,
}

/// Per-device streaming state used while partitioning generations.
#[derive(Default)]
struct DeviceState {
    generation_id: Option<i64>,
    unwrapper: Unwrapper,
    last_seq: BTreeMap<u8, i64>,
    boot_marker: Option<String>,
    boot_id: Option<i64>,
    last_continuous: Option<f64>,
}

fn now_secs() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

fn err<T>(code: &'static str, m: String) -> AppResult<T> {
    Err(AppError::new(code, m))
}

pub fn recover(db: &Connection) -> AppResult<()> {
    recover_pending(db).map_err(|e| AppError::new("db_error", e.to_string()))?;
    Ok(())
}

fn scale_cov(
    cov: &[f64],
    lf: f64,
    af: f64,
) -> AppResult<Cov6> {
    let mut m = [[0.0f64; 6]; 6];
    if cov.len() == 6 {
        for i in 0..3 {
            m[i][i] = cov[i] * lf * lf;
            m[i + 3][i + 3] = cov[i + 3] * af * af;
        }
    } else {
        for i in 0..6 {
            for j in 0..6 {
                let v = cov[i * 6 + j];
                let fi = if i < 3 { lf } else { af };
                let fj = if j < 3 { lf } else { af };
                m[i][j] = v * fi * fj;
            }
        }
        // symmetrize against noisy input
        for i in 0..6 {
            for j in (i + 1)..6 {
                let a = 0.5 * (m[i][j] + m[j][i]);
                m[i][j] = a;
                m[j][i] = a;
            }
        }
    }
    let c = Cov6(m);
    if !c.is_positive_definite() {
        return Err(AppError::singular(
            "gnss pose covariance is not positive definite",
        ));
    }
    Ok(c)
}

#[allow(clippy::too_many_arguments)]
fn finish_insert(
    db: &Connection,
    kind: PacketKind,
    p: &PacketInput,
    raw: &RawTime,
    units: &UnitSpec,
    coord_frame: &str,
    ds: &mut DeviceState,
    t: ContinuousTime,
    received_index: i64,
    report: &mut ImportReport,
) -> AppResult<()> {
    let gid = ds.generation_id.unwrap();

    // Duplicate detection is scoped to (device, seq): after a reboot seq may
    // legitimately restart, but those packets live in a *new* generation, so
    // matching also requires the same generation.  Content equality removes
    // ambiguity; overlapping-but-distinct packets are kept.
    let dup: Option<(i64, String)> = db
        .query_row(
            "SELECT id, payload_json FROM packets
             WHERE device_id=?1 AND seq=?2 AND generation_id=?3
             ORDER BY id LIMIT 1",
            params![p.device_id, p.seq, gid],
            |r| Ok((r.get(0)?, r.get::<_, String>(1)?)),
        )
        .ok();
    let payload_json = serde_json::to_string(p).map_err(|e| {
        AppError::new("encode_error", e.to_string())
    })?;
    let duplicate_of = match dup {
        Some((id, old_payload)) if old_payload == payload_json => Some(id),
        _ => None,
    };

    let summary = content_summary(kind, p, units);
    db.execute(
        "INSERT INTO packets
         (generation_id,kind,device_id,seq,boot_id,boot_marker,
          raw_sow,raw_week,leap_flag,time_scale,coord_frame,
          length_unit,angle_unit,continuous_time,received_index,
          duplicate_of,content_summary,payload_json)
         VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13,?14,?15,?16,?17,?18)",
        params![
            gid,
            match kind {
                PacketKind::Lidar => "lidar",
                PacketKind::Gnss => "gnss",
            },
            p.device_id,
            p.seq,
            p.boot_id,
            p.boot_marker,
            raw.sow,
            raw.week,
            raw.leap_second_flag as i64,
            raw.scale,
            coord_frame,
            units.length.label(),
            units.angle.label(),
            t.secs(),
            received_index,
            duplicate_of,
            summary,
            payload_json,
        ],
    )
    .map_err(fe)?;
    let pid = db.last_insert_rowid();

    if duplicate_of.is_some() {
        report.duplicates += 1;
    } else {
        report.accepted += 1;
    }

    if kind == PacketKind::Gnss {
        insert_pose(db, p, units, pid, gid, t.secs())?;
    }
    Ok(())
}

fn insert_pose(
    db: &Connection,
    p: &PacketInput,
    units: &UnitSpec,
    pid: i64,
    gid: i64,
    t: f64,
) -> AppResult<()> {
    let (px, py, pz) = match (p.px, p.py, p.pz) {
        (Some(x), Some(y), Some(z)) => (x, y, z),
        _ => return Err(AppError::new("bad_pose", "gnss packet needs px,py,pz")),
    };
    let lf = units.length.to_meters();
    let af = units.angle.to_radians();
    let q = match p.quat {
        Some(q) => {
            let q = Quat([q[0], q[1], q[2], q[3]]).normalize();
            if !q.0.iter().all(|v| v.is_finite()) {
                return Err(AppError::new("bad_pose", "non-finite quaternion"));
            }
            q
        }
        None => match (p.roll, p.pitch, p.yaw) {
            (Some(r), Some(pp), Some(yw)) => {
                Quat::from_euler_xyz(r * af, pp * af, yw * af)
            }
            _ => Quat::identity(),
        },
    };
    let cov = match &p.cov {
        Some(v) => scale_cov(v, lf, af)?,
        None => Cov6::diagonal([1e-4, 1e-4, 1e-4], [1e-6, 1e-6, 1e-6]),
    };
    db.execute(
        "INSERT INTO poses(packet_id,generation_id,t,qw,qx,qy,qz,tx,ty,tz,cov_json)
         VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11)",
        params![
            pid,
            gid,
            t,
            q.0[0],
            q.0[1],
            q.0[2],
            q.0[3],
            px * lf,
            py * lf,
            pz * lf,
            serde_json::to_string(&cov.0).unwrap(),
        ],
    )
    .map_err(fe)?;
    Ok(())
}

#[derive(Clone, Debug, Serialize)]
pub struct InterpPose {
    pub iso: Iso,
    pub cov: Cov6,
    pub gap: f64,
    pub left_packet_id: i64,
    pub right_packet_id: i64,
}

#[derive(Clone, Debug, Serialize)]
pub struct InterpError {
    pub reason: String,
}

fn nlerp(a: Quat, b: Quat, u: f64) -> Quat {
    let mut q = [0.0f64; 4];
    let dot = a.0[0] * b.0[0]
        + a.0[1] * b.0[1]
        + a.0[2] * b.0[2]
        + a.0[3] * b.0[3];
    let bb = if dot < 0.0 {
        [-b.0[0], -b.0[1], -b.0[2], -b.0[3]]
    } else {
        b.0
    };
    for i in 0..4 {
        q[i] = a.0[i] * (1.0 - u) + bb[i] * u;
    }
    Quat(q).normalize()
}

/// Interpolate body pose at time `t` using poses of one generation only.
/// Fails when the bracketing poses would span a gap larger than
/// `MAX_INTERP_GAP` or when `t` is outside observed pose coverage.
pub fn interpolate_pose(
    db: &Connection,
    generation_id: i64,
    t: f64,
) -> Result<InterpPose, InterpError> {
    let fail = |m: &str| InterpError {
        reason: m.to_string(),
    };
    let left = db
        .query_row(
            "SELECT packet_id,t,qw,qx,qy,qz,tx,ty,tz,cov_json FROM poses
             WHERE generation_id=?1 AND t<=?2 ORDER BY t DESC LIMIT 1",
            params![generation_id, t],
            pose_row,
        )
        .map_err(|_| fail("no pose at or before time (new generation edge?)"))?;
    let right = db
        .query_row(
            "SELECT packet_id,t,qw,qx,qy,qz,tx,ty,tz,cov_json FROM poses
             WHERE generation_id=?1 AND t>=?2 ORDER BY t ASC LIMIT 1",
            params![generation_id, t],
            pose_row,
        )
        .map_err(|_| fail("no pose at or after time"))?;
    if left.packet_id == right.packet_id {
        return Ok(InterpPose {
            iso: left.iso.clone(),
            cov: left.cov.clone(),
            gap: 0.0,
            left_packet_id: left.packet_id,
            right_packet_id: right.packet_id,
        });
    }
    let gap = right.t - left.t;
    if gap > MAX_INTERP_GAP {
        return Err(fail("pose gap exceeds allowed interpolation interval"));
    }
    if !(gap > 0.0) {
        return Err(fail("degenerate pose interval"));
    }
    let u = (t - left.t) / gap;
    let q = nlerp(left.iso.q, right.iso.q, u);
    let trans = vadd(
        vscale(left.iso.t, 1.0 - u),
        vscale(right.iso.t, u),
    );
    // Interpolated covariance: weighted average plus an uncertainty inflation
    // proportional to the bracketing gap (linearization drift).
    let mut m = [[0.0f64; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            m[i][j] = left.cov.0[i][j] * (1.0 - u)
                + right.cov.0[i][j] * u
                + (i == j) as i64 as f64 * gap * gap * 1e-3;
        }
    }
    Ok(InterpPose {
        iso: Iso::new(q, trans),
        cov: Cov6(m),
        gap,
        left_packet_id: left.packet_id,
        right_packet_id: right.packet_id,
    })
}

struct PoseRow {
    packet_id: i64,
    t: f64,
    iso: Iso,
    cov: Cov6,
}

fn pose_row(r: &rusqlite::Row<'_>) -> rusqlite::Result<PoseRow> {
    let cov_str: String = r.get("cov_json")?;
    let cov: [[f64; 6]; 6] =
        serde_json::from_str(&cov_str).unwrap_or_else(|_| [[0.0; 6]; 6]);
    Ok(PoseRow {
        packet_id: r.get("packet_id")?,
        t: r.get("t")?,
        iso: Iso::new(
            Quat([
                r.get("qw")?,
                r.get("qx")?,
                r.get("qy")?,
                r.get("qz")?,
            ]),
            [r.get("tx")?, r.get("ty")?, r.get("tz")?],
        ),
        cov: Cov6(cov),
    })
}

fn block_start(t: f64) -> f64 {
    (t / BLOCK_SECS).floor() * BLOCK_SECS
}

/// Deterministic signature of every input that can affect a block's derived
/// points: poses bracketing the window, edges valid in the window, the raw
/// packets in the window and the algorithm version.
fn block_signature(
    db: &Connection,
    gid: i64,
    start: f64,
    end: f64,
) -> rusqlite::Result<String> {
    let mut h = String::from(ALGO_VERSION);
    h.push('|');
    {
        let mut s = db.prepare(
            "SELECT id,t FROM poses WHERE generation_id=?1 AND t>=?2 AND t<=?3
             ORDER BY t,id",
        )?;
        let rows = s.query_map(params![gid, start - MAX_INTERP_GAP, end], |r| {
            Ok((r.get::<_, i64>(0)?, r.get::<_, f64>(1)?))
        })?;
        for row in rows {
            let (id, t) = row?;
            h.push_str(&format!("p{id}@{t:.6};"));
        }
    }
    {
        let mut s = db.prepare(
            "SELECT id,version,COALESCE(valid_from,-1e18),COALESCE(valid_to,1e18)
             FROM edges
             WHERE COALESCE(valid_from,-1e18)<?2 AND COALESCE(valid_to,1e18)>?1
             ORDER BY id,version",
        )?;
        let rows = s.query_map(params![start, end], |r| {
            Ok((
                r.get::<_, i64>(0)?,
                r.get::<_, i64>(1)?,
                r.get::<_, f64>(2)?,
                r.get::<_, f64>(3)?,
            ))
        })?;
        for row in rows {
            let (id, v, a, b) = row?;
            h.push_str(&format!("e{id}v{v}@{a:.3}-{b:.3};"));
        }
    }
    {
        let mut s = db.prepare(
            "SELECT id,seq,continuous_time,content_summary FROM packets
             WHERE generation_id=?1 AND continuous_time>=?2 AND continuous_time<?3
             ORDER BY id",
        )?;
        let rows = s.query_map(params![gid, start, end], |r| {
            Ok((
                r.get::<_, i64>(0)?,
                r.get::<_, i64>(1)?,
                r.get::<_, f64>(2)?,
                r.get::<_, String>(3)?,
            ))
        })?;
        for row in rows {
            let (id, sq, t, c) = row?;
            h.push_str(&format!("k{id}/{sq}@{t:.6}/{c};"));
        }
    }
    Ok(h)
}

/// Build/rebuild every stale derived block.  Blocks are created in
/// `pending`; a block only becomes `complete` once all of its points (and
/// their provenance) are committed inside one transaction.  A crash therefore
/// leaves no half-block masquerading as success — recovery deletes pending
/// blocks and resumes from the last complete one.
pub fn build_all(db: &Connection) -> AppResult<BuildReport> {
    recover(db)?;
    let mut report = BuildReport {
        blocks_complete: 0,
        blocks_rebuilt: 0,
        blocks_skipped: 0,
        points_aligned: 0,
        points_unaligned: 0,
    };

    let gids: Vec<i64> = {
        let mut s = db
            .prepare("SELECT id FROM generations ORDER BY created_order,id")
            .map_err(fe)?;
        let rows = s.query_map([], |r| r.get::<_, i64>(0)).map_err(fe)?;
        rows.collect::<rusqlite::Result<Vec<_>>>().map_err(fe)?
    };

    for gid in gids {
        let (tmin, tmax): (f64, f64) = db
            .query_row(
                "SELECT MIN(continuous_time), MAX(continuous_time)
                 FROM packets WHERE generation_id=?1",
                params![gid],
                |r| Ok((r.get(0)?, r.get(1)?)),
            )
            .map_err(fe)?;
        let mut start = block_start(tmin);
        while start <= tmax {
            let end = start + BLOCK_SECS;
            let sig = block_signature(db, gid, start, end).map_err(fe)?;
            let existing: Option<(i64, String, String)> = db
                .query_row(
                    "SELECT id,status,signature FROM blocks
                     WHERE generation_id=?1 AND start_t=?2",
                    params![gid, start],
                    |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
                )
                .ok();
            match existing {
                Some((_, status, old_sig))
                    if status == "complete" && old_sig == sig =>
                {
                    let (a, u) = block_point_counts(db, gid, start)?;
                    report.blocks_skipped += 1;
                    report.points_aligned += a;
                    report.points_unaligned += u;
                }
                _ => {
                    if existing.is_some() {
                        report.blocks_rebuilt += 1;
                    }
                    let (a, u) = build_block(db, gid, start, end, &sig)?;
                    report.blocks_complete += 1;
                    report.points_aligned += a;
                    report.points_unaligned += u;
                }
            }
            start = end;
        }
    }
    Ok(report)
}

fn block_point_counts(
    db: &Connection,
    gid: i64,
    start: f64,
) -> AppResult<(usize, usize)> {
    let (a, u): (i64, i64) = db
        .query_row(
            "SELECT
               SUM(CASE WHEN aligned=1 THEN 1 ELSE 0 END),
               SUM(CASE WHEN aligned=0 THEN 1 ELSE 0 END)
             FROM derived_points dp JOIN blocks b ON b.id=dp.block_id
             WHERE dp.generation_id=?1 AND b.start_t=?2",
            params![gid, start],
            |r| Ok((r.get::<_, Option<i64>>(0)?.unwrap_or(0),
                    r.get::<_, Option<i64>>(1)?.unwrap_or(0))),
        )
        .map_err(fe)?;
    Ok((a as usize, u as usize))
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Provenance {
    pub point: PointOrigin,
    pub chain: Vec<ChainHop>,
    pub candidates: Vec<CandidateRoute>,
    pub final_sigma_position_m: Option<[f64; 3]>,
    pub final_map_xyz: Option<[f64; 3]>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PointOrigin {
    pub packet_id: i64,
    pub generation_id: i64,
    pub point_index: usize,
    pub device_id: String,
    pub raw_seq: i64,
    pub raw_timestamp_sow: f64,
    pub raw_week: Option<i64>,
    pub time_scale: String,
    pub coord_frame: String,
    pub units: String,
    pub content_summary: String,
    pub raw_sensor_xyz: [f64; 3],
    pub si_sensor_xyz: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChainHop {
    pub from: String,
    pub to: String,
    pub arc_id: String,
    pub version: String,
    pub covariance_trace: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CandidateRoute {
    pub frames: Vec<String>,
    pub precision_cost: f64,
    pub version_score: i64,
    pub selected: bool,
    pub reason: String,
}

fn static_arcs_at(
    edges: &[Edge],
    t: f64,
) -> Vec<RouteArc> {
    // Choose the single highest applicable version per (source,target).
    let mut best: BTreeMap<(String, String), &Edge> = BTreeMap::new();
    for e in edges {
        if !graph::edge_valid_at(e, t) {
            continue;
        }
        best.entry((e.source.clone(), e.target.clone()))
            .and_modify(|cur| {
                if e.version > cur.version {
                    *cur = e;
                }
            })
            .or_insert(e);
    }
    best.into_iter()
        .map(|(_, e)| RouteArc {
            source: e.source.clone(),
            target: e.target.clone(),
            iso: e.iso,
            cov: e.cov.clone(),
            arc_id: format!("edge#{}@v{}", e.id, e.version),
            version_rank: e.version,
        })
        .collect()
}

fn body_frame_for(device_id: &str) -> String {
    format!("body/{device_id}")
}

fn lidar_frame_for(device_id: &str) -> String {
    format!("lidar/{device_id}")
}

#[derive(Clone)]
struct PendingPoint {
    packet_id: i64,
    device_id: String,
    seq: i64,
    raw_sow: f64,
    week: Option<i64>,
    scale: String,
    frame: String,
    units: String,
    summary: String,
    index: usize,
    t: f64,
    si: [f64; 3],
    raw_xyz: [f64; 3],
}

fn build_block(
    db: &Connection,
    gid: i64,
    start: f64,
    end: f64,
    sig: &str,
) -> AppResult<(usize, usize)> {
    // Delete any stale block at this window first (old invalidated version).
    db.execute(
        "DELETE FROM derived_points WHERE block_id IN
            (SELECT id FROM blocks WHERE generation_id=?1 AND start_t=?2)",
        params![gid, start],
    )
    .map_err(fe)?;
    db.execute(
        "DELETE FROM blocks WHERE generation_id=?1 AND start_t=?2",
        params![gid, start],
    )
    .map_err(fe)?;

    db.execute(
        "INSERT INTO blocks(generation_id,start_t,end_t,status,signature,built_at)
         VALUES(?1,?2,?3,'pending',?4,NULL)",
        params![gid, start, end, sig],
    )
    .map_err(fe)?;
    let block_id = db.last_insert_rowid();

    let edges = graph::all_edges(db).map_err(fe)?;

    // Gather non-duplicate lidar packets in the window.
    let mut points: Vec<PendingPoint> = Vec::new();
    {
        let mut s = db
            .prepare(
                "SELECT id,device_id,seq,raw_sow,raw_week,time_scale,coord_frame,
                        length_unit,continuous_time,payload_json,content_summary
                 FROM packets
                 WHERE generation_id=?1 AND kind='lidar'
                   AND continuous_time>=?2 AND continuous_time<?3
                   AND duplicate_of IS NULL
                 ORDER BY continuous_time,id",
            )
            .map_err(fe)?;
        let rows = s
            .query_map(params![gid, start, end], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    r.get::<_, String>(1)?,
                    r.get::<_, i64>(2)?,
                    r.get::<_, f64>(3)?,
                    r.get::<_, Option<i64>>(4)?,
                    r.get::<_, String>(5)?,
                    r.get::<_, String>(6)?,
                    r.get::<_, String>(7)?,
                    r.get::<_, f64>(8)?,
                    r.get::<_, String>(9)?,
                    r.get::<_, String>(10)?,
                ))
            })
            .map_err(fe)?;
        for row in rows {
            let (pid, dev, seq, sow, week, scale, frame, lunit, t, payload, summary) =
                row.map_err(fe)?;
            let parsed: PacketInput =
                serde_json::from_str(&payload).unwrap_or_else(|_| {
                    serde_json::from_value(serde_json::json!({"kind":"lidar",
                        "device_id":dev,"seq":seq,"timestamp":sow}))
                    .unwrap()
                });
            let lf = parsed
                .length_unit
                .as_deref()
                .and_then(LengthUnit::parse)
                .unwrap_or(LengthUnit::Meter)
                .to_meters();
            if let Some(pts) = &parsed.points {
                for (i, pt) in pts.iter().enumerate() {
                    points.push(PendingPoint {
                        packet_id: pid,
                        device_id: dev.clone(),
                        seq,
                        raw_sow: sow,
                        week,
                        scale: scale.clone(),
                        frame: frame.clone(),
                        units: lunit.clone(),
                        summary: summary.clone(),
                        index: i,
                        t,
                        si: [pt.x * lf, pt.y * lf, pt.z * lf],
                        raw_xyz: [pt.x, pt.y, pt.z],
                    });
                }
            }
        }
    }

    let mut aligned = 0usize;
    let mut unaligned = 0usize;

    for pp in &points {
        let (map_xyz, prov) = align_point(db, &edges, gid, pp)?;
        let prov_json = serde_json::to_string(&prov).unwrap();
        let is_aligned = map_xyz.is_some();
        db.execute(
            "INSERT INTO derived_points
             (packet_id,block_id,generation_id,point_index,t,
              sx,sy,sz,mx,my,mz,aligned,route_key,sigma_position,provenance_json)
             VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13,?14,?15)",
            params![
                pp.packet_id,
                block_id,
                gid,
                pp.index as i64,
                pp.t,
                pp.si[0],
                pp.si[1],
                pp.si[2],
                map_xyz.map(|m| m[0]),
                map_xyz.map(|m| m[1]),
                map_xyz.map(|m| m[2]),
                is_aligned as i64,
                prov.chain.first().map(|_| "map"),
                prov.final_sigma_position_m
                    .map(|a| a.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(","))
                    .unwrap_or_default(),
                prov_json,
            ],
        )
        .map_err(fe)?;
        if is_aligned {
            aligned += 1;
        } else {
            unaligned += 1;
        }
    }

    // Atomic promotion: pending -> complete is the single commit boundary.
    db.execute(
        "UPDATE blocks SET status='complete', built_at=?1 WHERE id=?2",
        params![now_secs(), block_id],
    )
    .map_err(fe)?;
    Ok((aligned, unaligned))
}

fn arc_version_label(arc_id: &str) -> String {
    arc_id.to_string()
}

fn align_point(
    db: &Connection,
    edges: &[Edge],
    gid: i64,
    pp: &PendingPoint,
) -> AppResult<(Option<[f64; 3]>, Provenance)> {
    let body = body_frame_for(&pp.device_id);
    let sensor = lidar_frame_for(&pp.device_id);

    let origin = PointOrigin {
        packet_id: pp.packet_id,
        generation_id: gid,
        point_index: pp.index,
        device_id: pp.device_id.clone(),
        raw_seq: pp.seq,
        raw_timestamp_sow: pp.raw_sow,
        raw_week: pp.week,
        time_scale: pp.scale.clone(),
        coord_frame: pp.frame.clone(),
        units: pp.units.clone(),
        content_summary: pp.summary.clone(),
        raw_sensor_xyz: pp.raw_xyz,
        si_sensor_xyz: pp.si,
    };

    let mut arcs = static_arcs_at(edges, pp.t);

    // Dynamic GNSS arc body -> map is only available when interpolation is
    // legal inside this generation and within the maximum allowed gap.
    let mut gnss_note: Option<String> = None;
    match interpolate_pose(db, gid, pp.t) {
        Ok(ip) => {
            arcs.push(RouteArc {
                source: body.clone(),
                target: "map".into(),
                iso: ip.iso,
                cov: ip.cov,
                arc_id: format!(
                    "gnss#g{}:{}->{}@t{:.4}(gap{:.3})",
                    gid, ip.left_packet_id, ip.right_packet_id, pp.t, ip.gap
                ),
                version_rank: 1,
            });
        }
        Err(e) => {
            gnss_note = Some(e.reason);
        }
    }

    let routes = graph::enumerate_routes(&arcs, &sensor, "map");
    let mut chain: Vec<ChainHop> = Vec::new();
    let mut candidates: Vec<CandidateRoute> = Vec::new();

    for (i, r) in routes.iter().enumerate() {
        let reason = if i == 0 {
            "selected: lowest propagated covariance trace, then version rule"
                .into()
        } else {
            let diff = r.precision_cost - routes[0].precision_cost;
            format!("candidate only: cost +{diff:.3e} (or older versions)")
        };
        candidates.push(CandidateRoute {
            frames: r.frames.clone(),
            precision_cost: r.precision_cost,
            version_score: r.version_score,
            selected: i == 0,
            reason,
        });
    }

    let Some(winner) = routes.first() else {
        let reason = gnss_note.unwrap_or_else(|| "no transform path to map".into());
        return Ok((
            None,
            Provenance {
                point: origin,
                chain: vec![ChainHop {
                    from: sensor,
                    to: "map".into(),
                    arc_id: "UNRESOLVED".into(),
                    version: reason.clone(),
                    covariance_trace: None,
                }],
                candidates: vec![],
                final_sigma_position_m: None,
                final_map_xyz: None,
            },
        ));
    };

    // Re-resolve the chosen arcs so we can propagate the point covariance.
    let winner_arcs: Vec<&RouteArc> = winner
        .steps
        .iter()
        .map(|st| {
            arcs.iter()
                .find(|a| a.arc_id == st.arc_id && a.source == st.from && a.target == st.to)
                .expect("step arc exists")
        })
        .collect();

    for st in &winner.steps {
        chain.push(ChainHop {
            from: st.from.clone(),
            to: st.to.clone(),
            arc_id: st.arc_id.clone(),
            version: arc_version_label(&st.arc_id),
            covariance_trace: Some(st.sigma_step_trace),
        });
    }

    let (map_xyz, sigma) = propagate_point(&winner_arcs, pp.si);
    Ok((
        Some(map_xyz),
        Provenance {
            point: origin,
            chain,
            candidates,
            final_sigma_position_m: Some(sigma),
            final_map_xyz: Some(map_xyz),
        },
    ))
}

/// Transform a sensor point through the route and propagate a 3x3 position
/// covariance.  The raw point is treated as exact (LiDAR range noise defaults
/// to a small floor); each edge contributes translation+rotation uncertainty.
fn propagate_point(
    arcs: &[&RouteArc],
    p0: [f64; 3],
) -> ([f64; 3], [f64; 3]) {
    // Point covariance accumulator (3x3).  Start with a small range floor.
    let mut cov = [
        [1e-6f64, 0.0, 0.0],
        [0.0, 1e-6, 0.0],
        [0.0, 0.0, 1e-6],
    ];
    let mut p = p0;
    let mut total = Iso::identity();
    for a in arcs {
        let r = a.iso.q.rotation_matrix();
        // New point: p' = R p + t.
        // Contribution of existing point covariance: R cov R^T.
        let rotated = cov3_rot(&r, &cov);
        // Edge uncertainty contribution for this specific lever arm:
        // translation block R_tt + lever-arm rotation sensitivity J C J^T.
        let lever = p; // local point before this edge
        let t_cov = cov3_block(&a.cov.0, 0, 0);
        let r_cov = cov3_block(&a.cov.0, 3, 3);
        let jr = crate::math::rot_jac_wrt_angle(&r, lever);
        let edge_cov = cov3_add(&t_cov, &cov3_rot(&jr, &r_cov));
        cov = cov3_add(&rotated, &edge_cov);
        p = a.iso.apply(p);
        total = total.compose(&a.iso);
    }
    let _ = total;
    (p, [cov[0][0].sqrt(), cov[1][1].sqrt(), cov[2][2].sqrt()])
}

fn cov3_block(m: &[[f64; 6]; 6], i0: usize, j0: usize) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = m[i0 + i][j0 + j];
        }
    }
    r
}

fn cov3_rot(r: &[[f64; 3]; 3], c: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let cr = mat3x3(r, c);
    let mut rt = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            rt[i][j] = r[j][i];
        }
    }
    mat3x3(&cr, &rt)
}

fn mat3x3(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

fn cov3_add(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][j] + b[i][j];
        }
    }
    r
}

/// Add a new edge version.  Returns (edge_id, version).
pub fn add_edge(db: &Connection, input: EdgeInput) -> AppResult<(i64, i64)> {
    if input.covariance.len() != 6 && input.covariance.len() != 36 {
        return Err(AppError::singular(
            "covariance must contain 6 (diagonal) or 36 entries",
        ));
    }
    let q = match input.quat {
        Some(q) => Quat([q[0], q[1], q[2], q[3]]),
        None => match input.euler_rpy {
            Some(rpy) => Quat::from_euler_xyz(rpy[0], rpy[1], rpy[2]),
            None => Quat::identity(),
        },
    };
    let iso = Iso::new(q, input.translation);
    let mut m = [[0.0f64; 6]; 6];
    if input.covariance.len() == 6 {
        for i in 0..6 {
            m[i][i] = input.covariance[i];
        }
    } else {
        for i in 0..6 {
            for j in 0..6 {
                m[i][j] = input.covariance[i * 6 + j];
            }
        }
    }
    let cov = Cov6(m);
    if !cov.is_positive_definite() {
        return Err(AppError::singular(
            "edge covariance is not positive definite",
        ));
    }
    if let (Some(a), Some(b)) = (input.valid_from, input.valid_to) {
        if a >= b {
            return Err(AppError::new(
                "bad_window",
                "valid_from must be smaller than valid_to",
            ));
        }
    }
    let origin = input
        .origin
        .clone()
        .unwrap_or_else(|| "manual".to_string());
    let id = graph::upsert_edge(
        db,
        &input.source,
        &input.target,
        input.valid_from,
        input.valid_to,
        &iso,
        &cov,
        &origin,
        now_secs(),
    )?;
    let version = db
        .query_row(
            "SELECT version FROM edges WHERE id=?1",
            params![id],
            |r| r.get(0),
        )
        .map_err(fe)?;
    invalidate_for_window(db, input.valid_from, input.valid_to);
    Ok((id, version))
}

/// Invalidate only derived blocks whose time window intersects the edge
/// validity window (local invalidation).  Their signatures would mismatch on
/// the next build anyway; this eagerly drops affected points so the UI shows
/// the recomputation boundary clearly.
pub fn invalidate_for_window(
    db: &Connection,
    from: Option<f64>,
    to: Option<f64>,
) -> usize {
    let lo = from.unwrap_or(f64::NEG_INFINITY);
    let hi = to.unwrap_or(f64::INFINITY);
    let affected: Vec<i64> = {
        let mut s = db.prepare(
            "SELECT id FROM blocks
             WHERE status='complete' AND start_t<?2 AND end_t>?1",
        ).unwrap();
        let rows = s.query_map(params![lo, hi], |r| r.get::<_, i64>(0)).unwrap();
        rows.flatten().collect::<Vec<_>>()
    };
    for id in &affected {
        let _ = db.execute(
            "DELETE FROM derived_points WHERE block_id=?1",
            params![id],
        );
        let _ = db.execute("DELETE FROM blocks WHERE id=?1", params![id]);
    }
    affected.len()
}

/// Convenience: apply a calibration revision (new edge version) and rebuild.
pub fn revise_and_build(db: &Connection, input: EdgeInput) -> AppResult<serde_json::Value> {
    let (id, version) = add_edge(db, input)?;
    let report = build_all(db)?;
    Ok(serde_json::json!({
        "edge_id": id,
        "version": version,
        "build": report,
    }))
}

#[derive(Serialize)]
pub struct StateView {
    pub generations: Vec<GenView>,
    pub edges: Vec<EdgeView>,
    pub poses: Vec<PoseView>,
    pub blocks: Vec<BlockView>,
    pub points: Vec<PointView>,
    pub gaps: Vec<GapView>,
    pub routes: Vec<RouteSummary>,
}

#[derive(Serialize)]
pub struct GenView {
    pub id: i64,
    pub device_id: String,
    pub reason: String,
    pub t0: Option<f64>,
    pub packet_count: i64,
}

#[derive(Serialize)]
pub struct EdgeView {
    pub id: i64,
    pub source: String,
    pub target: String,
    pub version: i64,
    pub valid_from: Option<f64>,
    pub valid_to: Option<f64>,
    pub origin: String,
    pub sigma_trace: f64,
}

#[derive(Serialize)]
pub struct PoseView {
    pub generation_id: i64,
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize)]
pub struct BlockView {
    pub id: i64,
    pub generation_id: i64,
    pub start: f64,
    pub end: f64,
    pub status: String,
}

#[derive(Serialize)]
pub struct PointView {
    pub id: i64,
    pub generation_id: i64,
    pub t: f64,
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub z: Option<f64>,
    pub aligned: bool,
    pub color_group: i64,
}

#[derive(Serialize)]
pub struct GapView {
    pub generation_id: i64,
    pub from_t: f64,
    pub to_t: f64,
    pub seconds: f64,
    pub kind: String,
}

#[derive(Serialize)]
pub struct RouteSummary {
    pub sensor: String,
    pub t: f64,
    pub selected: Vec<String>,
    pub cost: f64,
    pub candidates: Vec<CandidateRoute>,
}

pub fn state_view(db: &Connection) -> AppResult<StateView> {
    let generations = {
        let mut s = db
            .prepare(
                "SELECT g.id,g.device_id,g.reason,g.opened_at,
                        (SELECT COUNT(*) FROM packets p
                         WHERE p.generation_id=g.id) AS n
                 FROM generations g ORDER BY g.created_order,g.id",
            )
            .map_err(fe)?;
        let rows = s
            .query_map([], |r| {
                Ok(GenView {
                    id: r.get(0)?,
                    device_id: r.get(1)?,
                    reason: r.get(2)?,
                    t0: r.get(3)?,
                    packet_count: r.get(4)?,
                })
            })
            .map_err(fe)?;
        rows.collect::<rusqlite::Result<Vec<_>>>().map_err(fe)?
    };
    let edges = {
        let es = graph::all_edges(db).map_err(fe)?;
        es.into_iter()
            .map(|e| EdgeView {
                id: e.id,
                source: e.source,
                target: e.target,
                version: e.version,
                valid_from: e.valid_from,
                valid_to: e.valid_to,
                origin: e.origin,
                sigma_trace: e.cov.trace_cost(),
            })
            .collect()
    };
    let poses = {
        let mut s = db
            .prepare(
                "SELECT generation_id,t,tx,ty,tz FROM poses
                 ORDER BY generation_id,t",
            )
            .map_err(fe)?;
        let rows = s
            .query_map([], |r| {
                Ok(PoseView {
                    generation_id: r.get(0)?,
                    t: r.get(1)?,
                    x: r.get(2)?,
                    y: r.get(3)?,
                    z: r.get(4)?,
                })
            })
            .map_err(fe)?;
        rows.collect::<rusqlite::Result<Vec<_>>>().map_err(fe)?
    };
    let blocks = {
        let mut s = db
            .prepare(
                "SELECT id,generation_id,start_t,end_t,status
                 FROM blocks ORDER BY generation_id,start_t",
            )
            .map_err(fe)?;
        let rows = s
            .query_map([], |r| {
                Ok(BlockView {
                    id: r.get(0)?,
                    generation_id: r.get(1)?,
                    start: r.get(2)?,
                    end: r.get(3)?,
                    status: r.get(4)?,
                })
            })
            .map_err(fe)?;
        rows.collect::<rusqlite::Result<Vec<_>>>().map_err(fe)?
    };
    let points = {
        let mut s = db
            .prepare(
                "SELECT id,generation_id,t,mx,my,mz,aligned
                 FROM derived_points ORDER BY t,id",
            )
            .map_err(fe)?;
        let rows = s
            .query_map([], |r| {
                Ok(PointView {
                    id: r.get(0)?,
                    generation_id: r.get(1)?,
                    t: r.get(2)?,
                    x: r.get(3)?,
                    y: r.get(4)?,
                    z: r.get(5)?,
                    aligned: r.get::<_, i64>(6)? != 0,
                    color_group: r.get::<_, i64>(1)?,
                })
            })
            .map_err(fe)?;
        rows.collect::<rusqlite::Result<Vec<_>>>().map_err(fe)?
    };

    let gaps = compute_gaps(db)?;
    let routes = route_summaries(db)?;

    Ok(StateView {
        generations,
        edges,
        poses,
        blocks,
        points,
        gaps,
        routes,
    })
}

fn compute_gaps(db: &Connection) -> AppResult<Vec<GapView>> {
    let mut gaps = Vec::new();
    // Within-generation pose gaps that forbid interpolation.
    {
        let mut s = db
            .prepare(
                "SELECT generation_id,t FROM poses ORDER BY generation_id,t",
            )
            .map_err(fe)?;
        let rows = s
            .query_map([], |r| {
                Ok((r.get::<_, i64>(0)?, r.get::<_, f64>(1)?))
            })
            .map_err(fe)?;
        let mut prev: Option<(i64, f64)> = None;
        for row in rows {
            let (g, t) = row.map_err(fe)?;
            if let Some((pg, pt)) = prev {
                if pg == g && t - pt > MAX_INTERP_GAP {
                    gaps.push(GapView {
                        generation_id: g,
                        from_t: pt,
                        to_t: t,
                        seconds: t - pt,
                        kind: "pose_gap".into(),
                    });
                }
            }
            prev = Some((g, t));
        }
    }
    // Generation boundaries themselves are gaps the chain must not cross.
    {
        let mut s = db
            .prepare(
                "SELECT generation_id, MIN(continuous_time), MAX(continuous_time)
                 FROM packets GROUP BY generation_id
                 ORDER BY MIN(continuous_time)",
            )
            .map_err(fe)?;
        let rows = s
            .query_map([], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    r.get::<_, f64>(1)?,
                    r.get::<_, f64>(2)?,
                ))
            })
            .map_err(fe)?;
        let mut prev_end: Option<(i64, f64)> = None;
        for row in rows {
            let (g, lo, hi) = row.map_err(fe)?;
            if let Some((_pg, pe)) = prev_end {
                gaps.push(GapView {
                    generation_id: g,
                    from_t: pe,
                    to_t: lo,
                    seconds: lo - pe,
                    kind: "generation_boundary".into(),
                });
            }
            prev_end = Some((g, hi));
        }
    }
    gaps.sort_by(|a, b| {
        a.from_t
            .partial_cmp(&b.from_t)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(gaps)
}

/// One representative route summary per distinct (sensor,time) — taken from
/// the first aligned point of each sensor — for the browser path panel.
fn route_summaries(db: &Connection) -> AppResult<Vec<RouteSummary>> {
    let mut out = Vec::new();
    let mut s = db
        .prepare(
            "SELECT DISTINCT packet_id, t, provenance_json FROM derived_points
             WHERE aligned=1 ORDER BY t,id",
        )
        .map_err(fe)?;
    let rows = s
        .query_map([], |r| {
            Ok((
                r.get::<_, i64>(0)?,
                r.get::<_, f64>(1)?,
                r.get::<_, String>(2)?,
            ))
        })
        .map_err(fe)?;
    let mut seen: BTreeSet<String> = BTreeSet::new();
    for row in rows {
        let (_, t, js) = row.map_err(fe)?;
        let prov: Provenance = serde_json::from_str(&js).unwrap_or_else(|_| {
            Provenance {
                point: PointOrigin {
                    packet_id: 0,
                    generation_id: 0,
                    point_index: 0,
                    device_id: String::new(),
                    raw_seq: 0,
                    raw_timestamp_sow: 0.0,
                    raw_week: None,
                    time_scale: String::new(),
                    coord_frame: String::new(),
                    units: String::new(),
                    content_summary: String::new(),
                    raw_sensor_xyz: [0.0; 3],
                    si_sensor_xyz: [0.0; 3],
                },
                chain: vec![],
                candidates: vec![],
                final_sigma_position_m: None,
                final_map_xyz: None,
            }
        });
        let sensor = prov.point.coord_frame.clone();
        if !seen.insert(sensor.clone()) {
            continue;
        }
        let selected = prov
            .chain
            .iter()
            .flat_map(|h| {
                if h.from.is_empty() {
                    vec![]
                } else {
                    vec![h.from.clone(), h.to.clone()]
                }
            })
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let frames: Vec<String> = if prov.chain.is_empty() {
            vec![sensor.clone(), "map".into()]
        } else {
            let mut f = vec![prov.chain[0].from.clone()];
            for h in &prov.chain {
                f.push(h.to.clone());
            }
            f
        };
        let cost = prov
            .candidates
            .iter()
            .find(|c| c.selected)
            .map(|c| c.precision_cost)
            .unwrap_or(0.0);
        out.push(RouteSummary {
            sensor,
            t,
            selected: if selected.is_empty() { frames } else {
                rebuild_frame_order(&prov)
            },
            cost,
            candidates: prov.candidates,
        });
    }
    Ok(out)
}

fn rebuild_frame_order(prov: &Provenance) -> Vec<String> {
    let mut f = Vec::new();
    if let Some(first) = prov.chain.first() {
        f.push(first.from.clone());
    }
    for h in &prov.chain {
        f.push(h.to.clone());
    }
    f
}

pub fn point_provenance(db: &Connection, id: i64) -> AppResult<Provenance> {
    let js: String = db
        .query_row(
            "SELECT provenance_json FROM derived_points WHERE id=?1",
            params![id],
            |r| r.get(0),
        )
        .map_err(|_| AppError::not_found(format!("derived point {id}")))?;
    serde_json::from_str(&js)
        .map_err(|e| AppError::new("decode_error", e.to_string()))
}

fn parse_units(p: &PacketInput, kind: PacketKind) -> AppResult<UnitSpec> {
    let length = match &p.length_unit {
        None => LengthUnit::Meter,
        Some(s) => LengthUnit::parse(s)
            .ok_or_else(|| AppError::bad_unit(format!("unknown length unit '{s}'")))?,
    };
    let angle = match &p.angle_unit {
        None => AngleUnit::Radian,
        Some(s) => AngleUnit::parse(s)
            .ok_or_else(|| AppError::bad_unit(format!("unknown angle unit '{s}'")))?,
    };
    // Cross-check: a lidar packet reporting degree distances is almost
    // certainly a unit mix-up worth refusing outright.
    if kind == PacketKind::Lidar {
        if let Some(s) = &p.length_unit {
            if s.trim().to_ascii_lowercase() == "deg"
                || s.trim().to_ascii_lowercase() == "degree"
            {
                return Err(AppError::bad_unit(
                    "lidar distance cannot be expressed in degrees",
                ));
            }
        }
    }
    Ok(UnitSpec { length, angle })
}

fn content_summary(
    kind: PacketKind,
    p: &PacketInput,
    units: &UnitSpec,
) -> String {
    match kind {
        PacketKind::Lidar => {
            let n = p.points.as_ref().map(|v| v.len()).unwrap_or(0);
            format!(
                "lidar:{}pts({})",
                n,
                units.length.label()
            )
        }
        PacketKind::Gnss => format!(
            "gnss:pose({},{})",
            units.length.label(),
            units.angle.label()
        ),
    }
}

/// Import packets in *receipt order*.  Generation partitioning happens here,
/// before any sorting by timestamp, so reboot sequence resets, GPS week
/// rollovers and leap-second neighbourhoods are handled explicitly.
pub fn import_packets(
    db: &Connection,
    packets: Vec<PacketInput>,
) -> AppResult<ImportReport> {
    let mut report = ImportReport {
        accepted: 0,
        duplicates: 0,
        rejected: 0,
        new_generations: Vec::new(),
        errors: Vec::new(),
    };
    let mut state: BTreeMap<String, DeviceState> = BTreeMap::new();
    let mut order: i64 = db
        .query_row("SELECT COALESCE(MAX(received_index),0) FROM packets", [], |r| {
            r.get(0)
        })
        .unwrap_or(0);

    for p in packets {
        order += 1;
        if let Err(e) = import_one(db, p, order, &mut state, &mut report) {
            report.rejected += 1;
            report.errors.push(e.to_string());
        }
    }
    Ok(report)
}

fn open_generation(
    db: &Connection,
    device_id: &str,
    reason: &str,
    opened_at: f64,
    report: &mut ImportReport,
) -> AppResult<i64> {
    let order = db
        .query_row("SELECT COALESCE(MAX(created_order),0)+1 FROM generations", [], |r| {
            r.get::<_, i64>(0)
        })
        .map_err(fe)?;
    db.execute(
        "INSERT INTO generations(device_id,reason,opened_at,created_order)
         VALUES(?1,?2,?3,?4)",
        params![device_id, reason, opened_at, order],
    )
    .map_err(fe)?;
    let id = db.last_insert_rowid();
    report.new_generations.push(id);
    Ok(id)
}

fn fe(e: rusqlite::Error) -> AppError {
    AppError::new("db_error", e.to_string())
}

#[allow(clippy::too_many_arguments)]
fn import_one(
    db: &Connection,
    p: PacketInput,
    received_index: i64,
    state: &mut BTreeMap<String, DeviceState>,
    report: &mut ImportReport,
) -> AppResult<()> {
    let kind = PacketKind::parse(&p.kind)
        .ok_or_else(|| AppError::new("bad_kind", format!("unknown kind '{}'", p.kind)))?;
    if p.device_id.trim().is_empty() {
        return Err(AppError::new("bad_device", "device_id is required"));
    }
    let units = parse_units(&p, kind)?;
    if p.timestamp.is_nan() || p.timestamp < 0.0 {
        return Err(AppError::new("bad_time", "timestamp must be >= 0"));
    }
    if kind == PacketKind::Gnss {
        let cov_ok = match &p.cov {
            None => true,
            Some(v) => v.len() == 36 || v.len() == 6,
        };
        if !cov_ok {
            return Err(AppError::singular(
                "gnss covariance must have 6 or 36 entries",
            ));
        }
    }

    let raw = RawTime {
        sow: p.timestamp,
        week: p.week,
        leap_second_flag: p.leap_second_flag,
        scale: p.time_scale.clone().unwrap_or_else(|| "gps".into()),
    };
    let coord_frame = p
        .coord_frame
        .clone()
        .unwrap_or_else(|| match kind {
            PacketKind::Lidar => format!("lidar/{}", p.device_id),
            PacketKind::Gnss => format!("body/{}", p.device_id),
        });

    let ds = state.entry(p.device_id.clone()).or_default();

    // ---- Explicit generation boundaries, evaluated from receipt order ----
    let mut reason: Option<&str> = None;
    if ds.generation_id.is_none() {
        reason = Some("first_packet");
    } else if p.boot_marker.is_some()
        && ds.boot_marker.is_some()
        && p.boot_marker != ds.boot_marker
    {
        reason = Some("boot_marker_changed");
    } else if p.boot_id.is_some()
        && ds.boot_id.is_some()
        && p.boot_id != ds.boot_id
    {
        reason = Some("boot_id_changed");
    } else {
        // Implicit reboot: a far sequence reset that is *not* a duplicated or
        // merely late packet (those have a small seq delta and near timestamps).
        let kind_key = if kind == PacketKind::Gnss { 0 } else { 1 };
        if let Some(last_seq) = ds.last_seq.get(&kind_key) {
            let far_reset = p.seq + 10 < *last_seq;
            let near_old =
                (p.timestamp - ds.last_continuous.unwrap_or(p.timestamp)).abs() <= 2.0;
            if far_reset && !near_old {
                reason = Some("seq_reset_reboot");
            }
        }
    }

    // Open a new generation immediately when an explicit reboot was seen, so
    // the time unwrapper starts fresh and cannot smear across the boundary.
    if let Some(r) = reason {
        let gid = open_generation(db, &p.device_id, r, p.timestamp, report)?;
        ds.generation_id = Some(gid);
        ds.unwrapper = Unwrapper::new();
        ds.last_seq = BTreeMap::new();
        ds.last_continuous = None;
    }

    // Unwrap time within the (possibly fresh) generation.
    let resolved = ds.unwrapper.resolve(&raw);

    // Implicit time boundary: a *large* backward jump which the unwrapper
    // could not explain as a week rollover or a leap second means a new
    // acquisition generation.  Small backward steps are ordinary late
    // packets and stay in the current generation (they are never used to
    // bridge interpolation gaps).
    if reason.is_none()
        && is_generation_break(resolved.backward_seconds)
        && !resolved.week_rollover
        && !resolved.leap_adjusted
    {
        let gid =
            open_generation(db, &p.device_id, "time_discontinuity", resolved.t.secs(), report)?;
        ds.generation_id = Some(gid);
        ds.unwrapper = Unwrapper::new();
        ds.last_seq = BTreeMap::new();
        ds.last_continuous = None;
        let r2 = ds.unwrapper.resolve(&raw);
        finish_insert(
            db, kind, &p, &raw, &units, &coord_frame, ds, r2.t, received_index, report,
        )?;
    } else {
        finish_insert(
            db, kind, &p, &raw, &units, &coord_frame, ds, resolved.t, received_index, report,
        )?;
    }

    // Running sequence high-water mark: a late/overlapping packet never
    // lowers it, so a genuine reset is still detectable afterwards.
    let kind_key = if kind == PacketKind::Gnss { 0 } else { 1 };
    ds.last_seq
        .entry(kind_key)
        .and_modify(|s| *s = (*s).max(p.seq))
        .or_insert(p.seq);
    ds.boot_marker = p.boot_marker.clone();
    ds.boot_id = p.boot_id;
    ds.last_continuous = Some(ds.last_continuous.map_or(raw.sow, |s| s.max(raw.sow)));
    Ok(())
}
