//! SQLite persistence. Raw packets are immutable source data; derived
//! map points and derived blocks are explicit caches that can be
//! invalidated and rebuilt.

use rusqlite::Connection;
use std::sync::Mutex;

pub struct Db(pub Mutex<Connection>);

pub const SCHEMA: &str = r#"
PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS streams (
    id          INTEGER PRIMARY KEY,
    device_id   TEXT NOT NULL UNIQUE,
    kind        TEXT NOT NULL CHECK (kind IN ('lidar','gnss')),
    coord_frame TEXT NOT NULL,
    unit        TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS packets (
    id              INTEGER PRIMARY KEY,
    stream_id       INTEGER NOT NULL REFERENCES streams(id),
    generation      INTEGER NOT NULL,
    seq             INTEGER NOT NULL,
    raw_ts          TEXT NOT NULL,
    raw_clock       TEXT NOT NULL,
    gps_ms          INTEGER NOT NULL,
    coord_frame     TEXT NOT NULL,
    unit            TEXT NOT NULL,
    content_summary TEXT NOT NULL,
    content_hash    INTEGER NOT NULL,
    restart_flag    INTEGER NOT NULL DEFAULT 0,
    flag            TEXT NOT NULL,
    received_order  INTEGER NOT NULL,
    payload_json    TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_packets_stream_time ON packets(stream_id, gps_ms);
CREATE INDEX IF NOT EXISTS idx_packets_gen ON packets(stream_id, generation);

CREATE TABLE IF NOT EXISTS frames (
    id          INTEGER PRIMARY KEY,
    packet_id   INTEGER NOT NULL UNIQUE REFERENCES packets(id),
    stream_id   INTEGER NOT NULL REFERENCES streams(id),
    generation  INTEGER NOT NULL,
    time_ms     INTEGER NOT NULL,
    canonical   INTEGER NOT NULL,
    point_count INTEGER NOT NULL,
    unit        TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS raw_points (
    id        INTEGER PRIMARY KEY,
    frame_id  INTEGER NOT NULL REFERENCES frames(id),
    idx       INTEGER NOT NULL,
    x         REAL NOT NULL,
    y         REAL NOT NULL,
    z         REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_raw_points_frame ON raw_points(frame_id);

-- Pose samples in the parent frame ("map"), child frame is the body.
CREATE TABLE IF NOT EXISTS poses (
    id          INTEGER PRIMARY KEY,
    packet_id   INTEGER NOT NULL REFERENCES packets(id),
    stream_id   INTEGER NOT NULL REFERENCES streams(id),
    generation  INTEGER NOT NULL,
    time_ms     INTEGER NOT NULL,
    tx REAL, ty REAL, tz REAL,
    qw REAL, qx REAL, qy REAL, qz REAL
);


CREATE TABLE IF NOT EXISTS edges (
    id           INTEGER PRIMARY KEY,
    name         TEXT NOT NULL,
    from_frame   TEXT NOT NULL,
    to_frame     TEXT NOT NULL,
    version      INTEGER NOT NULL,
    supersedes   INTEGER,
    valid_start  INTEGER,
    valid_end    INTEGER,
    dynamic      INTEGER NOT NULL DEFAULT 0,
    pose_stream  INTEGER REFERENCES streams(id),
    tx REAL, ty REAL, tz REAL,
    qw REAL, qx REAL, qy REAL, qz REAL,
    cov_json     TEXT,
    created_order INTEGER NOT NULL,
    UNIQUE(name, version)
);
CREATE INDEX IF NOT EXISTS idx_edges_frames ON edges(from_frame, to_frame);

CREATE TABLE IF NOT EXISTS blocks (
    id           INTEGER PRIMARY KEY,
    frame_id     INTEGER NOT NULL REFERENCES frames(id),
    source_frame TEXT NOT NULL,
    target_frame TEXT NOT NULL,
    status       TEXT NOT NULL CHECK (status IN ('pending','building','done','blocked','stale')),
    input_hash   INTEGER NOT NULL,
    edge_versions TEXT NOT NULL,
    error        TEXT,
    attempt      INTEGER NOT NULL DEFAULT 0,
    UNIQUE(frame_id, source_frame, target_frame)
);

CREATE TABLE IF NOT EXISTS map_points (
    id        INTEGER PRIMARY KEY,
    block_id  INTEGER NOT NULL REFERENCES blocks(id),
    raw_point_id INTEGER NOT NULL REFERENCES raw_points(id),
    idx       INTEGER NOT NULL,
    x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
    cov_json  TEXT NOT NULL,
    provenance_json TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_map_points_block ON map_points(block_id);
"#;

pub fn open(path: &str) -> rusqlite::Result<Db> {
    let conn = Connection::open(path)?;
    conn.pragma_update(None, "busy_timeout", 5000)?;
    conn.execute_batch(SCHEMA)?;
    // Correct pose index (the annotated CREATE INDEX above references a
    // missing column on some drafts); (re)create defensively.
    conn.execute_batch(
        "CREATE INDEX IF NOT EXISTS idx_poses_time ON poses(stream_id, time_ms);",
    )?;
    Ok(Db(Mutex::new(conn)))
}

pub fn open_memory() -> rusqlite::Result<Db> {
    let conn = Connection::open_in_memory()?;
    conn.execute_batch(SCHEMA)?;
    conn.execute_batch("CREATE INDEX IF NOT EXISTS idx_poses_time ON poses(stream_id, time_ms);")?;
    Ok(Db(Mutex::new(conn)))
}
