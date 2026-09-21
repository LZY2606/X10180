//! SQLite persistence.  Every fact about an acquisition is stored as an
//! immutable row; derived caches (blocks/points) are explicitly marked and can
//! be invalidated and rebuilt without touching imported packets.

use rusqlite::{params, Connection};
use std::sync::Mutex;

pub const SCHEMA_VERSION: i64 = 1;
pub const ALGO_VERSION: &str = "pjm-1";

pub struct Db(pub Mutex<Connection>);

pub fn conn(path: &str) -> rusqlite::Result<Connection> {
    let c = Connection::open(path)?;
    c.pragma_update(None, "journal_mode", "WAL")?;
    c.pragma_update(None, "foreign_keys", "ON")?;
    c.pragma_update(None, "busy_timeout", 10_000)?;
    Ok(c)
}

pub fn init(c: &Connection) -> rusqlite::Result<()> {
    c.execute_batch(
        r#"
CREATE TABLE IF NOT EXISTS meta (
  key TEXT PRIMARY KEY,
  value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS generations (
  id INTEGER PRIMARY KEY,
  device_id TEXT NOT NULL,
  reason TEXT NOT NULL,
  opened_at REAL,
  created_order INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS packets (
  id INTEGER PRIMARY KEY,
  generation_id INTEGER NOT NULL REFERENCES generations(id),
  kind TEXT NOT NULL,
  device_id TEXT NOT NULL,
  seq INTEGER NOT NULL,
  boot_id INTEGER,
  boot_marker TEXT,
  raw_sow REAL NOT NULL,
  raw_week INTEGER,
  leap_flag INTEGER NOT NULL,
  time_scale TEXT NOT NULL,
  coord_frame TEXT NOT NULL,
  length_unit TEXT NOT NULL,
  angle_unit TEXT NOT NULL,
  continuous_time REAL NOT NULL,
  received_index INTEGER NOT NULL,
  duplicate_of INTEGER REFERENCES packets(id),
  content_summary TEXT NOT NULL,
  payload_json TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_packets_gen_time ON packets(generation_id, continuous_time);
CREATE INDEX IF NOT EXISTS idx_packets_device_seq ON packets(device_id, seq);

CREATE TABLE IF NOT EXISTS frames (
  name TEXT PRIMARY KEY,
  kind TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS edges (
  id INTEGER PRIMARY KEY,
  source TEXT NOT NULL,
  target TEXT NOT NULL,
  version INTEGER NOT NULL,
  supersedes INTEGER,
  valid_from REAL,
  valid_to REAL,
  qw REAL NOT NULL, qx REAL NOT NULL, qy REAL NOT NULL, qz REAL NOT NULL,
  tx REAL NOT NULL, ty REAL NOT NULL, tz REAL NOT NULL,
  cov_json TEXT NOT NULL,
  origin TEXT NOT NULL,
  created_at REAL NOT NULL,
  UNIQUE(source, target, version)
);
CREATE INDEX IF NOT EXISTS idx_edges_pair ON edges(source, target);

CREATE TABLE IF NOT EXISTS poses (
  id INTEGER PRIMARY KEY,
  packet_id INTEGER NOT NULL REFERENCES packets(id),
  generation_id INTEGER NOT NULL,
  t REAL NOT NULL,
  qw REAL, qx REAL, qy REAL, qz REAL,
  tx REAL, ty REAL, tz REAL,
  cov_json TEXT
);
CREATE INDEX IF NOT EXISTS idx_poses_gen_t ON poses(generation_id, t);

CREATE TABLE IF NOT EXISTS blocks (
  id INTEGER PRIMARY KEY,
  generation_id INTEGER NOT NULL,
  start_t REAL NOT NULL,
  end_t REAL NOT NULL,
  status TEXT NOT NULL,           -- complete | pending
  signature TEXT NOT NULL,
  built_at REAL
);
CREATE INDEX IF NOT EXISTS idx_blocks_gen ON blocks(generation_id, start_t);

CREATE TABLE IF NOT EXISTS derived_points (
  id INTEGER PRIMARY KEY,
  packet_id INTEGER NOT NULL REFERENCES packets(id),
  block_id INTEGER REFERENCES blocks(id),
  generation_id INTEGER NOT NULL,
  point_index INTEGER NOT NULL,
  t REAL NOT NULL,
  sx REAL NOT NULL, sy REAL NOT NULL, sz REAL NOT NULL,
  mx REAL, my REAL, mz REAL,
  aligned INTEGER NOT NULL,
  route_key TEXT,
  sigma_position REAL,
  provenance_json TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_points_block ON derived_points(block_id);
CREATE INDEX IF NOT EXISTS idx_points_time ON derived_points(generation_id, t);
"#,
    )?;

    let v: i64 = c
        .query_row(
            "SELECT COUNT(*) FROM meta WHERE key='schema_version'",
            [],
            |r| r.get(0),
        )
        .unwrap_or(0);
    if v == 0 {
        c.execute(
            "INSERT INTO meta(key, value) VALUES('schema_version', ?1)",
            params![SCHEMA_VERSION.to_string()],
        )?;
        c.execute(
            "INSERT INTO meta(key, value) VALUES('seeded', '0')",
            [],
        )?;
    }
    Ok(())
}

impl Db {
    pub fn open(path: &str) -> crate::model::AppResult<Self> {
        let c = conn(path).map_err(|e| {
            crate::model::AppError::new("db_error", format!("open {path}: {e}"))
        })?;
        init(&c).map_err(|e| {
            crate::model::AppError::new("db_error", format!("init: {e}"))
        })?;
        Ok(Db(Mutex::new(c)))
    }

    pub fn meta(&self, key: &str) -> Option<String> {
        self.0
            .lock()
            .unwrap()
            .query_row("SELECT value FROM meta WHERE key=?1", params![key], |r| {
                r.get::<_, String>(0)
            })
            .ok()
    }

    pub fn set_meta(&self, key: &str, value: &str) {
        self.0
            .lock()
            .unwrap()
            .execute(
                "INSERT INTO meta(key,value) VALUES(?1,?2)
                 ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                params![key, value],
            )
            .unwrap();
    }

    pub fn lock(&self) -> std::sync::MutexGuard<'_, Connection> {
        self.0.lock().unwrap()
    }
}

/// Recover from a crash: any block left in `pending` (a half-built block must
/// never be treated as success) is deleted together with its points, so the
/// next build starts it from scratch.  `complete` blocks are reused.
pub fn recover_pending(c: &Connection) -> rusqlite::Result<usize> {
    let ids: Vec<i64> = {
        let mut stmt = c.prepare("SELECT id FROM blocks WHERE status='pending'")?;
        let rows = stmt.query_map([], |r| r.get::<_, i64>(0))?;
        rows.collect::<rusqlite::Result<Vec<_>>>()?
    };
    for id in ids {
        c.execute("DELETE FROM derived_points WHERE block_id=?1", params![id])?;
        c.execute("DELETE FROM blocks WHERE id=?1", params![id])?;
    }
    Ok(0)
}

/// Make sure the canonical frames exist.
pub fn ensure_frames(c: &Connection, names: &[(&str, &str)]) -> rusqlite::Result<()> {
    for (name, kind) in names {
        c.execute(
            "INSERT INTO frames(name, kind) VALUES(?1, ?2)
             ON CONFLICT(name) DO NOTHING",
            params![name, kind],
        )?;
    }
    Ok(())
}
