//! Application services: ingest raw packets (immutable), assign
//! generations, manage versioned transforms with partial invalidation,
//! and build derived blocks atomically with crash recovery.

use crate::db::Db;
use crate::graph::{self, Edge, PoseSample};
use crate::math::{self, Mat6, Rigid};
use crate::time::{self, PacketFlag, RawPacket};
use rusqlite::{params, Connection};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;

pub const MAX_POSE_INTERVAL_MS: i64 = 250;
pub const MAX_PATH_HOPS: usize = 8;
pub const POINT_NOISE: f64 = 1e-6;

#[derive(Clone)]
pub struct App {
    pub db: Arc<Db>,
}

#[derive(Deserialize)]
pub struct StreamInput {
    pub device_id: String,
    pub kind: String,
    pub coord_frame: String,
    pub unit: String,
}

#[derive(Deserialize, Serialize, Default)]
pub struct PacketInput {
    pub device_id: String,
    pub seq: u32,
    pub seq_modulus: Option<u32>,
    pub restart_flag: Option<bool>,
    /// One of the raw clock encodings below.
    pub clock: ClockInput,
    pub coord_frame: String,
    pub unit: String,
    pub content_summary: String,
    /// LiDAR points in the packet's native unit.
    #[serde(default)]
    pub points: Vec<[f64; 3]>,
    /// GNSS pose, meters in the map frame convention: translation + quat.
    #[serde(default)]
    pub pose: Option<PoseInput>,
}

#[derive(Deserialize, Serialize, Default)]
pub struct ClockInput {
    pub kind: String, // "gps_week" | "unix_ms" | "gps_ms"
    #[serde(default)]
    pub week10: Option<u32>,
    #[serde(default)]
    pub ms_of_week: Option<f64>,
    #[serde(default)]
    pub unix_ms: Option<i64>,
    #[serde(default)]
    pub gps_ms: Option<i64>,
    #[serde(default)]
    pub leap_second_occurred: Option<bool>,
}

#[derive(Deserialize, Serialize, Default)]
pub struct PoseInput {
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub qw: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

#[derive(Deserialize)]
pub struct EdgeInput {
    pub name: String,
    pub from_frame: String,
    pub to_frame: String,
    #[serde(default)]
    pub valid_start: Option<i64>,
    #[serde(default)]
    pub valid_end: Option<i64>,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub qw: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    /// 6x6 flat row-major covariance.
    #[serde(default)]
    pub covariance: Option<Vec<f64>>,
}

#[derive(Serialize, Debug)]
pub struct EdgeResult {
    pub id: i64,
    pub version: i64,
    pub invalidated_blocks: i64,
}

#[derive(Serialize, Debug)]
pub struct CycleError {
    pub error: String,
    pub cycle: graph::CycleReport,
}

fn flag_str(f: PacketFlag) -> &'static str {
    match f {
        PacketFlag::Normal => "normal",
        PacketFlag::GenerationRestart => "generation_restart",
        PacketFlag::GenerationGap => "generation_gap",
        PacketFlag::Late => "late",
        PacketFlag::Duplicate => "duplicate",
        PacketFlag::Overlap => "overlap",
    }
}

fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

impl App {
    pub fn new(db: Arc<Db>) -> Self {
        let app = App { db };
        app.recover_blocks();
        app
    }

    /// On startup / after a crash no `building` block may be treated as
    /// successful: return them to `pending` so the next run restarts from
    /// a complete block.
    pub fn recover_blocks(&self) {
        let c = self.db.0.lock().unwrap();
        c.execute(
            "UPDATE blocks SET status='pending', attempt=attempt+1,
                error='recovered interrupted build' WHERE status='building'",
            [],
        )
        .unwrap();
    }

    pub fn ensure_stream(&self, input: &StreamInput) -> i64 {
        let c = self.db.0.lock().unwrap();
        loop {
            if let Ok(id) = c.query_row(
                "SELECT id FROM streams WHERE device_id=?1",
                params![input.device_id],
                |r| r.get::<_, i64>(0),
            ) {
                return id;
            }
            c.execute(
                "INSERT INTO streams(device_id,kind,coord_frame,unit) VALUES(?1,?2,?3,?4)",
                params![input.device_id, input.kind, input.coord_frame, input.unit],
            )
            .unwrap();
        }
    }

    fn stream_info(c: &Connection, device_id: &str) -> (i64, String, String, String) {
        c.query_row(
            "SELECT id,kind,coord_frame,unit FROM streams WHERE device_id=?1",
            params![device_id],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
        )
        .unwrap()
    }

    fn normalize_time(&self, c: &Connection, stream_id: i64, clock: &ClockInput) -> Result<i64, String> {
        match clock.kind.as_str() {
            "gps_ms" => clock.gps_ms.ok_or_else(|| "missing gps_ms".into()),
            "unix_ms" => {
                let u = clock.unix_ms.ok_or("missing unix_ms")?;
                Ok(time::unix_ms_to_gps_ms(u, clock.leap_second_occurred.unwrap_or(false)))
            }
            "gps_week" => {
                let w = clock.week10.ok_or("missing week10")?;
                let ms = clock.ms_of_week.ok_or("missing ms_of_week")?;
                let anchor = c
                    .query_row(
                        "SELECT gps_ms FROM packets WHERE stream_id=?1 ORDER BY received_order DESC LIMIT 1",
                        params![stream_id],
                        |r| r.get::<_, i64>(0),
                    )
                    .ok();
                Ok(match anchor {
                    Some(a) => time::unwrap_gps_week(w, ms, a),
                    None => time::unwrap_gps_week_first(w, ms),
                })
            }
            other => Err(format!("unknown clock kind: {}", other)),
        }
    }

    /// Ingest one packet, assign its generation relative to prior packets
    /// of the same stream, and persist the immutable raw record plus any
    /// frame / pose extraction.
    pub fn ingest_packet(&self, input: &PacketInput) -> Result<i64, String> {
        if input.coord_frame.trim().is_empty() {
            return Err("coord_frame is required".into());
        }
        let known_units = ["m", "meter", "meters", "mm", "cm", "rad", "deg"];
        if !known_units.contains(&input.unit.as_str()) {
            return Err(format!("unknown unit '{}'; expected one of m/mm/cm", input.unit));
        }

        let mut c = self.db.0.lock().unwrap();
        let (stream_id, kind, _stream_frame, stream_unit) =
            App::stream_info(&c, &input.device_id);
        if kind != "lidar" && kind != "gnss" {
            return Err(format!("stream {} has unsupported kind {}", input.device_id, kind));
        }
        if input.coord_frame != _stream_frame {
            return Err(format!(
                "coordinate frame mismatch: stream {} declares {}, packet says {}",
                input.device_id, _stream_frame, input.coord_frame
            ));
        }
        if input.unit != stream_unit {
            return Err(format!(
                "unit mismatch: stream {} declares {}, packet says {}",
                input.device_id, stream_unit, input.unit
            ));
        }

        let gps_ms = self.normalize_time(&c, stream_id, &input.clock)?;
        let modulus = input.seq_modulus.unwrap_or(65536).max(1);

        // Load prior packets in receive order for generation assignment.
        let mut prior: Vec<RawPacket> = c
            .prepare("SELECT gps_ms,seq,content_hash,restart_flag FROM packets WHERE stream_id=?1 ORDER BY received_order")
            .unwrap()
            .query_map(params![stream_id], |r| {
                Ok(RawPacket {
                    time_ms: r.get(0)?,
                    seq: r.get::<_, i64>(1)? as u32,
                    seq_modulus: modulus,
                    restart_flag: r.get::<_, i64>(3)? != 0,
                    content_hash: r.get::<_, i64>(2)? as u64,
                })
            })
            .unwrap()
            .map(|r| r.unwrap())
            .collect();

        let payload = serde_json::to_string(input).unwrap();
        // Content identity excludes transport metadata (restart flag,
        // clock encoding): the same measurement replayed through a
        // different path is still the same content.
        let content_key = serde_json::json!({
            "device": input.device_id,
            "coord_frame": input.coord_frame,
            "unit": input.unit,
            "summary": input.content_summary,
            "points": input.points,
            "pose": input.pose,
        });
        let hash = fnv1a(content_key.to_string().as_bytes());
        prior.push(RawPacket {
            time_ms: gps_ms,
            seq: input.seq % modulus,
            seq_modulus: modulus,
            restart_flag: input.restart_flag.unwrap_or(false),
            content_hash: hash,
        });
        let assigned = time::assign_generations(&prior);
        let mine = assigned.last().unwrap();
        let generation = mine.generation as i64;
        let flag = flag_str(mine.flag);

        let raw_ts = match input.clock.kind.as_str() {
            "gps_week" => format!(
                "week10={},msow={}",
                input.clock.week10.unwrap_or(0),
                input.clock.ms_of_week.unwrap_or(0.0)
            ),
            "unix_ms" => format!("unix_ms={}", input.clock.unix_ms.unwrap_or(0)),
            _ => format!("gps_ms={}", gps_ms),
        };
        let received_order = c
            .query_row(
                "SELECT COALESCE(MAX(received_order)+1,0) FROM packets WHERE stream_id=?1",
                params![stream_id],
                |r| r.get::<_, i64>(0),
            )
            .unwrap();

        let is_pure_duplicate = mine.flag == PacketFlag::Duplicate;
        let tx = c.transaction().unwrap();
        let packet_id = {
            tx.execute(
                "INSERT INTO packets(stream_id,generation,seq,raw_ts,raw_clock,gps_ms,
                    coord_frame,unit,content_summary,content_hash,restart_flag,flag,received_order,payload_json)
                 VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,
?13,?14)",
                params![
                    stream_id, generation, input.seq as i64, raw_ts, input.clock.kind, gps_ms,
                    input.coord_frame, input.unit, input.content_summary, hash as i64,
                    input.restart_flag.unwrap_or(false) as i64, flag, received_order, payload
                ],
            )
            .unwrap();
            let id = tx.last_insert_rowid();
            if is_pure_duplicate {
                // Raw duplicate must still be durably persisted, but it
                // spawns no frame/pose extraction.
                tx.commit().unwrap();
                return Ok(id);
            }
            id
        };

        if kind == "lidar" && !input.points.is_empty() {
            // Canonical frame selection for overlap conflicts: lowest
            // content hash wins deterministically.
            let canonical = {
                let conflict: i64 = tx
                    .query_row(
                        "SELECT COUNT(*) FROM packets p JOIN frames f ON f.packet_id=p.id
                         WHERE p.stream_id=?1 AND p.generation=?2 AND p.seq=?3",
                        params![stream_id, generation, input.seq as i64],
                        |r| r.get(0),
                    )
                    .unwrap_or(0);
                conflict == 0
            };
            tx.execute(
                "INSERT INTO frames(packet_id,stream_id,generation,time_ms,canonical,point_count,unit)
                 VALUES(?1,?2,?3,?4,?5,?6,?7)",
                params![packet_id, stream_id, generation, gps_ms, canonical as i64,
                         input.points.len() as i64, input.unit],
            )
            .unwrap();
            let frame_id = tx.last_insert_rowid();
            for (i, p) in input.points.iter().enumerate() {
                // Normalize mm/cm into meters; raw points retain the
                // native unit column, stored values are meters.
                let scale = match input.unit.as_str() {
                    "mm" => 0.001,
                    "cm" => 0.01,
                    _ => 1.0,
                };
                tx.execute(
                    "INSERT INTO raw_points(frame_id,idx,x,y,z) VALUES(?1,?2,?3,?4,?5)",
                    params![frame_id, i as i64, p[0] * scale, p[1] * scale, p[2] * scale],
                )
                .unwrap();
            }
            enqueue_frame_blocks(&tx, frame_id);
        }

        if kind == "gnss" {
            if let Some(pose) = &input.pose {
                tx.execute(
                    "INSERT INTO poses(packet_id,stream_id,generation,time_ms,
                        tx,ty,tz,qw,qx,qy,qz) VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11)",
                    params![packet_id, stream_id, generation, gps_ms,
                        pose.tx, pose.ty, pose.tz, pose.qw, pose.qx, pose.qy, pose.qz],
                )
                .unwrap();
            }
        }

        tx.commit().unwrap();
        Ok(packet_id)
    }
}

fn enqueue_frame_blocks(c: &Connection, frame_id: i64) {
    // Default derived target: body-named frame -> "map". The source frame
    // is read from the frame's stream coord_frame.
    let (stream_id, coord_frame, time_ms): (i64, String, i64) = c
        .query_row(
            "SELECT f.stream_id, s.coord_frame, f.time_ms FROM frames f
             JOIN streams s ON s.id=f.stream_id WHERE f.id=?1",
            params![frame_id],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        )
        .unwrap();
    let _ = (stream_id, time_ms);
    c.execute(
        "INSERT OR IGNORE INTO blocks(frame_id,source_frame,target_frame,status,input_hash,edge_versions)
         VALUES(?1,?2,'map','pending',0,'')",
        params![frame_id, coord_frame],
    )
    .unwrap();
}

fn parse_cov(v: &Option<Vec<f64>>) -> Result<Mat6, String> {
    match v {
        None => {
            let mut c = Mat6::zero();
            for i in 0..6 {
                c.a[i][i] = 1e-4;
            }
            Ok(c)
        }
        Some(v) if v.len() == 36 => {
            let c = Mat6::from_flat(v);
            math::validate_covariance(&c)?;
            Ok(c)
        }
        Some(_) => Err("covariance must be 36 numbers (6x6 row major)".into()),
    }
}

pub(crate) fn edge_from_row(r: &rusqlite::Row) -> rusqlite::Result<Edge> {
    Ok(Edge {
        id: r.get(0)?,
        name: r.get(1)?,
        from_frame: r.get(2)?,
        to_frame: r.get(3)?,
        version: r.get(4)?,
        supersedes: r.get(5)?,
        valid_start: r.get(6)?,
        valid_end: r.get(7)?,
        dynamic: r.get::<_, i64>(8)? != 0,
        pose_stream: r.get(9)?,
        transform: Rigid {
            t: [r.get(10)?, r.get(11)?, r.get(12)?],
            q: math::qnorm([r.get(13)?, r.get(14)?, r.get(15)?, r.get(16)?]),
        },
        covariance: serde_json::from_str(&r.get::<_, String>(17)?).unwrap_or(Mat6::zero()),
        created_order: r.get(18)?,
    })
}

pub(crate) const EDGE_COLS: &str = "id,name,from_frame,to_frame,version,supersedes,valid_start,valid_end,\
    dynamic,pose_stream,tx,ty,tz,qw,qx,qy,qz,cov_json,created_order";

impl App {
    pub fn list_edges(&self) -> Vec<Edge> {
        let c = self.db.0.lock().unwrap();
        let mut s = c.prepare(&format!("SELECT {} FROM edges ORDER BY id", EDGE_COLS)).unwrap();
        let edges = s.query_map([], edge_from_row).unwrap().map(|r| r.unwrap()).collect();
        edges
    }

    /// Add a static transform. A calibration revision is a new version of
    /// the same edge name: it supersedes the previous version and only
    /// invalidates derived blocks whose time falls inside its validity
    /// window. A new static edge that closes a frame loop is rejected with
    /// the loop path and accumulated residual.
    pub fn add_edge(&self, input: &EdgeInput) -> Result<EdgeResult, CycleError> {
        let cov = match parse_cov(&input.covariance) {
            Ok(c) => c,
            Err(e) => return Err(CycleError { error: e, cycle: empty_cycle() }),
        };
        let q = math::qnorm([input.qw, input.qx, input.qy, input.qz]);

        let mut c = self.db.0.lock().unwrap();
        let prev: Option<(i64, i64)> = c
            .query_row(
                "SELECT id,version FROM edges WHERE name=?1 ORDER BY version DESC LIMIT 1",
                params![input.name],
                |r| Ok((r.get(0)?, r.get(1)?)),
            )
            .ok();
        let version = prev.map_or(1, |(_, v)| v + 1);
        let created = c
            .query_row("SELECT COALESCE(MAX(created_order)+1,0) FROM edges", [], |r| r.get::<_, i64>(0))
            .unwrap();

        let candidate = Edge {
            id: -1,
            name: input.name.clone(),
            from_frame: input.from_frame.clone(),
            to_frame: input.to_frame.clone(),
            version,
            supersedes: prev.map(|(id, _)| id),
            valid_start: input.valid_start,
            valid_end: input.valid_end,
            dynamic: false,
            pose_stream: None,
            transform: Rigid { t: [input.tx, input.ty, input.tz], q },
            covariance: cov.clone(),
            created_order: created,
        };

        // Cycle check only against *other* static edges. A revision of an
        // existing edge replaces it, so its previous version is excluded.
        let mut static_edges: Vec<Edge> = {
            let mut s = c
                .prepare(&format!("SELECT {} FROM edges WHERE dynamic=0 ORDER BY id", EDGE_COLS))
                .unwrap();
            s.query_map([], edge_from_row).unwrap().map(|r| r.unwrap()).collect()
        };
        if let Some((prev_id, _)) = prev {
            static_edges.retain(|e| e.id != prev_id);
        }
        if let Some(cycle) = graph::detect_cycle(&static_edges, &candidate) {
            return Err(CycleError {
                error: "transform rejected: closes a frame loop".into(),
                cycle,
            });
        }

        let tx = c.transaction().unwrap();
        tx.execute(
            "INSERT INTO edges(name,from_frame,to_frame,version,supersedes,valid_start,valid_end,
                dynamic,pose_stream,tx,ty,tz,qw,qx,qy,qz,cov_json,created_order)
             VALUES(?1,?2,?3,?4,?5,?6,?7,0,NULL,?8,?9,?10,?11,?12,?13,?14,?15,?16)",
            params![
                input.name, input.from_frame, input.to_frame, version,
                prev.map(|(id, _)| id), input.valid_start, input.valid_end,
                input.tx, input.ty, input.tz, q[0], q[1], q[2], q[3],
                serde_json::to_string(&cov).unwrap(), created
            ],
        )
        .unwrap();
        let edge_id = tx.last_insert_rowid();
        if prev.is_some() {
            // Close the previous version's validity window at the new one.
            tx.execute(
                "UPDATE edges SET valid_end=COALESCE(?1, valid_end) WHERE id=?2",
                params![input.valid_start, prev.unwrap().0],
            )
            .unwrap();
        }

        // Partial invalidation: only done blocks covering a frame time
        // inside the revised window become stale. Blocks outside keep
        // being served from cache.
        let invalidated = tx
            .execute(
                "UPDATE blocks SET status='stale'
                 WHERE status='done' AND frame_id IN (
                    SELECT f.id FROM frames f WHERE
                        (?1 IS NULL OR f.time_ms >= ?1)
                    AND (?2 IS NULL OR f.time_ms < ?2)
                 )",
                params![input.valid_start, input.valid_end],
            )
            .unwrap();
        // Stale blocks need rebuilding: queue them as pending only when
        // their inputs now resolve; recompute handles that.
        tx.execute("UPDATE blocks SET status='pending' WHERE status='stale'", [])
            .unwrap();
        tx.commit().unwrap();
        Ok(EdgeResult { id: edge_id, version, invalidated_blocks: invalidated as i64 })
    }

    /// Register a dynamic edge backed by a GNSS stream's interpolated
    /// poses (body -> map). Dynamic edges never participate in static
    /// cycle rejection.
    pub fn add_dynamic_edge(
        &self,
        name: &str,
        from_frame: &str,
        to_frame: &str,
        pose_stream_device: &str,
        cov: Mat6,
        valid_start: Option<i64>,
        valid_end: Option<i64>,
    ) -> Result<i64, String> {
        let c = self.db.0.lock().unwrap();
        let stream_id: i64 = c
            .query_row("SELECT id FROM streams WHERE device_id=?1", params![pose_stream_device], |r| r.get(0))
            .map_err(|_| format!("unknown pose stream {}", pose_stream_device))?;
        let version: i64 = c
            .query_row("SELECT COALESCE(MAX(version)+1,1) FROM edges WHERE name=?1", params![name], |r| r.get(0))
            .unwrap();
        let created: i64 = c
            .query_row("SELECT COALESCE(MAX(created_order)+1,0) FROM edges", [], |r| r.get(0))
            .unwrap();
        c.execute(
            "INSERT INTO edges(name,from_frame,to_frame,version,supersedes,valid_start,valid_end,
                dynamic,pose_stream,tx,ty,tz,qw,qx,qy,qz,cov_json,created_order)
             VALUES(?1,?2,?3,?4,NULL,?5,?6,1,?7,0,0,0,1,0,0,0,?8,?9)",
            params![name, from_frame, to_frame, version, valid_start, valid_end,
                stream_id, serde_json::to_string(&cov).unwrap(), created],
        )
        .unwrap();
        Ok(c.last_insert_rowid())
    }
}

fn empty_cycle() -> graph::CycleReport {
    graph::CycleReport {
        frames: vec![],
        edge_ids: vec![],
        residual: graph::TransformDto {
            tx: 0.0, ty: 0.0, tz: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        },
        residual_translation_norm: 0.0,
        residual_rotation_rad: 0.0,
    }
}

#[derive(Serialize, serde::Deserialize, Clone)]
pub struct ProvenanceHop {
    pub edge: String,
    pub edge_id: i64,
    pub version: i64,
    pub from_frame: String,
    pub to_frame: String,
    pub reversed: bool,
    pub interpolated: bool,
    pub time_ms: Option<i64>,
    pub covariance_trace_after: f64,
}

#[derive(Serialize)]
pub struct PointDetail {
    pub map_point_id: i64,
    pub frame_id: i64,
    pub block_id: i64,
    pub raw: [f64; 3],
    pub map: [f64; 3],
    pub point_covariance: [[f64; 3]; 3],
    pub chain: Vec<ProvenanceHop>,
    pub total_covariance: Vec<f64>,
    pub edge_versions: Vec<i64>,
}

#[derive(Serialize)]
pub struct BuildReport {
    pub built: i64,
    pub blocked: i64,
    pub pending: i64,
    pub details: Vec<BlockStatus>,
}

#[derive(Serialize, Clone)]
pub struct BlockStatus {
    pub block_id: i64,
    pub frame_id: i64,
    pub status: String,
    pub error: Option<String>,
}

impl App {
    fn load_edges(&self, c: &Connection) -> (Vec<Edge>, Vec<Edge>) {
        let all: Vec<Edge> = {
            let mut s = c.prepare(&format!("SELECT {} FROM edges ORDER BY id", EDGE_COLS)).unwrap();
            s.query_map([], edge_from_row).unwrap().map(|r| r.unwrap()).collect()
        };
        let stat = all.iter().filter(|e| !e.dynamic).cloned().collect();
        let dynm = all.iter().filter(|e| e.dynamic).cloned().collect();
        (stat, dynm)
    }

    fn load_poses(&self, c: &Connection) -> HashMap<i64, Vec<PoseSample>> {
        let mut map: HashMap<i64, Vec<PoseSample>> = HashMap::new();
        let mut s = c
            .prepare("SELECT stream_id,time_ms,generation,tx,ty,tz,qw,qx,qy,qz FROM poses ORDER BY time_ms")
            .unwrap();
        let rows = s
            .query_map([], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    PoseSample {
                        time_ms: r.get(1)?,
                        generation: r.get(2)?,
                        transform: Rigid {
                            t: [r.get(3)?, r.get(4)?, r.get(5)?],
                            q: math::qnorm([r.get(6)?, r.get(7)?, r.get(8)?, r.get(9)?]),
                        },
                    },
                ))
            })
            .unwrap();
        for r in rows {
            let (sid, sample) = r.unwrap();
            map.entry(sid).or_default().push(sample);
        }
        map
    }

    /// Rebuild every pending/stale block. Each block is flipped to
    /// `building` before any derived row is written and only committed as
    /// `done` inside one transaction; a crash in the middle leaves it
    /// `building`, which startup recovery returns to `pending`.
    pub fn recompute(&self) -> BuildReport {
        // Reset stale -> pending first.
        {
            let c = self.db.0.lock().unwrap();
            c.execute("UPDATE blocks SET status='pending' WHERE status='stale'", []).unwrap();
        }
        let mut built = 0i64;
        let mut blocked = 0i64;
        let mut details = Vec::new();

        loop {
            // Deterministic order: earliest frame first.
            let todo: Option<(i64, i64, i64, String, String)> = {
                let c = self.db.0.lock().unwrap();
                c.query_row(
                    "SELECT b.id,b.frame_id,f.time_ms,b.source_frame,b.target_frame FROM blocks b
                     JOIN frames f ON f.id=b.frame_id
                     WHERE b.status='pending' ORDER BY f.time_ms,b.id LIMIT 1",
                    [],
                    |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?, r.get(4)?)),
                )
                .ok()
            };
            let (block_id, frame_id, time_ms, source_frame, target_frame) = match todo {
                Some(v) => v,
                None => break,
            };

            // Mark building in its own immediate transaction.
            {
                let c = self.db.0.lock().unwrap();
                c.execute("UPDATE blocks SET status='building',attempt=attempt+1 WHERE id=?1",
                          params![block_id]).unwrap();
            }

            let outcome = self.build_block(block_id, frame_id, time_ms, &source_frame, &target_frame);
            let c = self.db.0.lock().unwrap();
            match outcome {
                Ok(()) => {
                    c.execute("UPDATE blocks SET status='done',error=NULL WHERE id=?1",
                              params![block_id]).unwrap();
                    built += 1;
                    details.push(BlockStatus { block_id, frame_id, status: "done".into(), error: None });
                }
                Err(e) => {
                    // Missing pose/path today => blocked, not failed: a
                    // later packet can unblock it.
                    let status = if e.contains("no transform path")
                        || e.contains("CrossesGeneration")
                        || e.contains("OutOfRange")
                        || e.contains("IntervalTooLarge")
                        || e.contains("NoSamples")
                    {
                        blocked += 1;
                        "blocked"
                    } else {
                        blocked += 1;
                        "blocked"
                    };
                    c.execute("UPDATE blocks SET status=?1,error=?2 WHERE id=?3",
                              params![status, e, block_id]).unwrap();
                    details.push(BlockStatus { block_id, frame_id, status: status.into(), error: Some(e) });
                }
            }
        }

        let pending = {
            let c = self.db.0.lock().unwrap();
            c.query_row("SELECT COUNT(*) FROM blocks WHERE status IN ('pending','building')", [],
                        |r| r.get::<_, i64>(0)).unwrap()
        };
        BuildReport { built, blocked, pending, details }
    }

    fn build_block(
        &self,
        block_id: i64,
        frame_id: i64,
        time_ms: i64,
        source_frame: &str,
        target_frame: &str,
    ) -> Result<(), String> {
        // Snapshot inputs.
        let (static_edges, dynamic_edges, poses, points, canonical, frame_unit) = {
            let c = self.db.0.lock().unwrap();
            let (s, d) = self.load_edges(&c);
            let p = self.load_poses(&c);
            let canonical: i64 = c
                .query_row("SELECT canonical FROM frames WHERE id=?1", params![frame_id], |r| r.get(0))
                .unwrap();
            let unit: String = c
                .query_row("SELECT unit FROM frames WHERE id=?1", params![frame_id], |r| r.get(0))
                .unwrap();
            let pts: Vec<(i64, f64, f64, f64)> = {
                let mut st = c.prepare("SELECT id,x,y,z FROM raw_points WHERE frame_id=?1 ORDER BY idx").unwrap();
                st.query_map(params![frame_id], |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)))
                    .unwrap().map(|r| r.unwrap()).collect()
            };
            (s, d, p, pts, canonical, unit)
        };
        let _ = frame_unit;

        if canonical == 0 {
            // Non-canonical overlap frame: keep block but derive nothing.
            let c = self.db.0.lock().unwrap();
            c.execute("DELETE FROM map_points WHERE block_id=?1", params![block_id]).unwrap();
            return Ok(());
        }

        let paths = graph::find_paths(
            &static_edges,
            &dynamic_edges,
            &poses,
            source_frame,
            target_frame,
            time_ms,
            MAX_POSE_INTERVAL_MS,
            MAX_PATH_HOPS,
        )?;
        let chosen = paths.iter().find(|p| p.selected).ok_or("no selected path")?;

        // Resolve composed transform + accumulated covariance by walking
        // the selected hops, recording the trace after each version.
        let mut total_xf = Rigid::identity();
        let mut total_cov = Mat6::zero();
        let mut chain: Vec<ProvenanceHop> = Vec::new();
        for hop in &chosen.hops {
            let hop_xf = Rigid {
                t: [hop.tx, hop.ty, hop.tz],
                q: [hop.qw, hop.qx, hop.qy, hop.qz],
            };
            // The hop transform is the already-resolved transform at the
            // frame time (interpolated for dynamic edges); only covariance
            // metadata is read from the edge definition.
            let edge_cov = {
                let e = static_edges.iter().chain(dynamic_edges.iter())
                    .find(|e| e.id == hop.edge_id).unwrap();
                e.covariance.clone()
            };
            total_cov = math::compose_cov(&hop_xf, &edge_cov, &total_cov);
            total_xf = hop_xf.compose(&total_xf);
            chain.push(ProvenanceHop {
                edge: hop.name.clone(),
                edge_id: hop.edge_id,
                version: hop.version,
                from_frame: hop.from_frame.clone(),
                to_frame: hop.to_frame.clone(),
                reversed: hop.reversed,
                interpolated: hop.interpolated,
                time_ms: hop.time_ms,
                covariance_trace_after: total_cov.trace(),
            });
        }

        let input_hash = fnv1a(
            format!("{}:{}:{:?}", frame_id, chosen.hops.len(),
                chosen.hops.iter().map(|h| (h.edge_id, h.version)).collect::<Vec<_>>())
            .as_bytes(),
        );
        let edge_versions: Vec<i64> = chosen.hops.iter().map(|h| h.version).collect();
        let provenance = serde_json::json!({
            "chain": chain,
            "total_covariance": total_cov.to_flat(),
            "edge_versions": edge_versions,
            "rank_reason": chosen.rank_reason,
            "candidate_count": paths.len(),
        });

        // Atomic derived write.
        let mut c = self.db.0.lock().unwrap();
        let tx = c.transaction().unwrap();
        tx.execute("DELETE FROM map_points WHERE block_id=?1", params![block_id]).unwrap();
        for (raw_id, x, y, z) in &points {
            let raw = [*x, *y, *z];
            let map = total_xf.apply(raw);
            let pcov = math::point_covariance(&total_xf, &total_cov, raw, POINT_NOISE);
            tx.execute(
                "INSERT INTO map_points(block_id,raw_point_id,idx,x,y,z,cov_json,provenance_json)
                 VALUES(?1,?2,?3,?4,?5,?6,?7,?8)",
                params![
                    block_id, raw_id, 0, map[0], map[1], map[2],
                    serde_json::to_string(&pcov).unwrap(),
                    provenance.to_string()
                ],
            )
            .unwrap();
        }
        tx.execute(
            "UPDATE blocks SET input_hash=?1,edge_versions=?2,error=NULL WHERE id=?3",
            params![input_hash as i64,
                serde_json::to_string(&edge_versions).unwrap(), block_id],
        )
        .unwrap();
        tx.commit().unwrap();
        Ok(())
    }

    pub fn point_detail(&self, map_point_id: i64) -> Option<PointDetail> {
        let c = self.db.0.lock().unwrap();
        let row = c
            .query_row(
                "SELECT mp.id,mp.block_id,b.frame_id,rp.x,rp.y,rp.z,mp.x,mp.y,mp.z,
                        mp.cov_json,mp.provenance_json
                 FROM map_points mp
                 JOIN blocks b ON b.id=mp.block_id
                 JOIN raw_points rp ON rp.id=mp.raw_point_id
                 WHERE mp.id=?1",
                params![map_point_id],
                |r| {
                    Ok((
                        r.get::<_, i64>(0)?,
                        r.get::<_, i64>(1)?,
                        r.get::<_, i64>(2)?,
                        r.get::<_, f64>(3)?,
                        r.get::<_, f64>(4)?,
                        r.get::<_, f64>(5)?,
                        r.get::<_, f64>(6)?,
                        r.get::<_, f64>(7)?,
                        r.get::<_, f64>(8)?,
                        r.get::<_, String>(9)?,
                        r.get::<_, String>(10)?,
                    ))
                },
            )
            .ok()?;
        let (id, block_id, frame_id, rx, ry, rz, mx, my, mz, cov_json, prov_json) = row;
        let chain: Vec<ProvenanceHop> = serde_json::from_value(
            serde_json::from_str::<serde_json::Value>(&prov_json).unwrap()["chain"].clone(),
        )
        .unwrap_or_default();
        let total: Vec<f64> = serde_json::from_value(
            serde_json::from_str::<serde_json::Value>(&prov_json).unwrap()["total_covariance"].clone(),
        )
        .unwrap_or_default();
        let versions: Vec<i64> = serde_json::from_value(
            serde_json::from_str::<serde_json::Value>(&prov_json).unwrap()["edge_versions"].clone(),
        )
        .unwrap_or_default();
        Some(PointDetail {
            map_point_id: id,
            frame_id,
            block_id,
            raw: [rx, ry, rz],
            map: [mx, my, mz],
            point_covariance: serde_json::from_str(&cov_json).unwrap(),
            chain,
            total_covariance: total,
            edge_versions: versions,
        })
    }
}


impl App {
    pub fn paths_at(&self, from: &str, to: &str, time_ms: i64)
        -> Result<Vec<graph::FoundPath>, String>
    {
        let c = self.db.0.lock().unwrap();
        let (stat, dynm) = self.load_edges(&c);
        let poses = self.load_poses(&c);
        graph::find_paths(&stat, &dynm, &poses, from, to, time_ms,
                          MAX_POSE_INTERVAL_MS, MAX_PATH_HOPS)
    }
}
