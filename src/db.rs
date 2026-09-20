//! SQLite 持久层。
//!
//! 设计原则：
//! - `packet*` / `raw_point` / `pose_sample` / `edge` 是导入与用户标定的**原始事实**，
//!   转换结果永远不回写这些表；
//! - `block` / `derived_point` / `point_chain_step` 是**派生缓存**，
//!   按块原子写入，半块永远不会处于 ready 状态。

use rusqlite::{params, Connection, OptionalExtension};
use std::sync::Mutex;

#[derive(Debug, Clone)]
pub struct Db(pub std::sync::Arc<Mutex<Connection>>);

pub const SCHEMA_VERSION: i64 = 1;

fn add_column_if_missing(conn: &Connection, table: &str, col: &str, ty: &str) -> anyhow::Result<()> {
    let exists: bool = conn
        .prepare(&format!("PRAGMA table_info({table})"))?
        .query_map([], |r| r.get::<_, String>(1))?
        .filter_map(Result::ok)
        .any(|name| name == col);
    if !exists {
        conn.execute_batch(&format!("ALTER TABLE {table} ADD COLUMN {col} {ty};"))?;
    }
    // 旧数据回填一次。
    if !exists {
        conn.execute(
            &format!("UPDATE {table} SET seq_head=(SELECT MAX(seq) FROM packet WHERE gen_id={table}.id) WHERE {table}.seq_head IS NULL AND ?1='generation'"),
            params![table],
        )?;
    }
    Ok(())
}

pub fn open(path: &str) -> anyhow::Result<Db> {
    let conn = Connection::open(path)?;
    conn.pragma_update(None, "journal_mode", "WAL")?;
    conn.pragma_update(None, "foreign_keys", "ON")?;
    conn.pragma_update(None, "busy_timeout", 10_000)?;
    migrate(&conn)?;
    Ok(Db(std::sync::Arc::new(Mutex::new(conn))))
}

pub fn open_in_memory() -> anyhow::Result<Db> {
    let conn = Connection::open_in_memory()?;
    conn.pragma_update(None, "foreign_keys", "ON")?;
    migrate(&conn)?;
    Ok(Db(std::sync::Arc::new(Mutex::new(conn))))
}

fn migrate(conn: &Connection) -> anyhow::Result<()> {
    conn.execute_batch(
        r#"
        CREATE TABLE IF NOT EXISTS meta (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );

        CREATE TABLE IF NOT EXISTS generation (
            id INTEGER PRIMARY KEY,
            device_id TEXT NOT NULL,
            kind TEXT NOT NULL CHECK(kind IN ('lidar','pose')),
            seq_start INTEGER NOT NULL,
            t_start REAL,
            t_end REAL,
            note TEXT NOT NULL DEFAULT ''
        );

        CREATE TABLE IF NOT EXISTS packet (
            id INTEGER PRIMARY KEY,
            device_id TEXT NOT NULL,
            kind TEXT NOT NULL CHECK(kind IN ('lidar','pose')),
            seq INTEGER NOT NULL,
            gen_id INTEGER NOT NULL REFERENCES generation(id),
            raw_time_json TEXT NOT NULL,
            time_desc TEXT NOT NULL,
            unix_time REAL,
            coord_system TEXT NOT NULL,
            unit TEXT NOT NULL,
            content_summary TEXT NOT NULL,
            received_order INTEGER NOT NULL,
            is_duplicate INTEGER NOT NULL DEFAULT 0,
            is_late INTEGER NOT NULL DEFAULT 0,
            duplicate_of INTEGER REFERENCES packet(id),
            raw_blob_len INTEGER NOT NULL DEFAULT 0
        );
        CREATE INDEX IF NOT EXISTS idx_packet_dev ON packet(device_id, kind, unix_time);
        CREATE INDEX IF NOT EXISTS idx_packet_gen ON packet(gen_id);

        CREATE TABLE IF NOT EXISTS raw_point (
            id INTEGER PRIMARY KEY,
            packet_id INTEGER NOT NULL REFERENCES packet(id),
            idx_in_packet INTEGER NOT NULL,
            x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
            intensity REAL NOT NULL DEFAULT 0.0,
            UNIQUE(packet_id, idx_in_packet)
        );

        CREATE TABLE IF NOT EXISTS pose_sample (
            id INTEGER PRIMARY KEY,
            packet_id INTEGER NOT NULL UNIQUE REFERENCES packet(id),
            qx REAL NOT NULL, qy REAL NOT NULL, qz REAL NOT NULL, qw REAL NOT NULL,
            tx REAL NOT NULL, ty REAL NOT NULL, tz REAL NOT NULL,
            cov_json TEXT NOT NULL
        );

        CREATE TABLE IF NOT EXISTS edge (
            id INTEGER PRIMARY KEY,
            key TEXT NOT NULL,
            version INTEGER NOT NULL,
            kind TEXT NOT NULL CHECK(kind IN ('calib','map')),
            source_frame TEXT NOT NULL,
            target_frame TEXT NOT NULL,
            qx REAL NOT NULL, qy REAL NOT NULL, qz REAL NOT NULL, qw REAL NOT NULL,
            tx REAL NOT NULL, ty REAL NOT NULL, tz REAL NOT NULL,
            cov_json TEXT NOT NULL,
            valid_from REAL NOT NULL,
            valid_to REAL NOT NULL,
            supersedes INTEGER,
            created_at REAL NOT NULL,
            active INTEGER NOT NULL DEFAULT 1,
            UNIQUE(key, version)
        );
        CREATE INDEX IF NOT EXISTS idx_edge_active ON edge(active);

        -- 派生缓存 ----------------------------------------------------------
        CREATE TABLE IF NOT EXISTS block (
            id INTEGER PRIMARY KEY,
            packet_id INTEGER NOT NULL UNIQUE REFERENCES packet(id),
            gen_id INTEGER NOT NULL REFERENCES generation(id),
            status TEXT NOT NULL CHECK(status IN ('building','ready','error')),
            fingerprint TEXT NOT NULL,
            error TEXT NOT NULL DEFAULT '',
            built_at REAL
        );

        CREATE TABLE IF NOT EXISTS derived_point (
            id INTEGER PRIMARY KEY,
            block_id INTEGER NOT NULL REFERENCES block(id) ON DELETE CASCADE,
            raw_point_id INTEGER NOT NULL REFERENCES raw_point(id),
            x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
            cov_xx REAL NOT NULL, cov_yy REAL NOT NULL, cov_zz REAL NOT NULL,
            path_score REAL NOT NULL,
            path_edge_ids TEXT NOT NULL,
            UNIQUE(raw_point_id, block_id)
        );

        CREATE TABLE IF NOT EXISTS point_chain_step (
            id INTEGER PRIMARY KEY,
            derived_point_id INTEGER NOT NULL REFERENCES derived_point(id) ON DELETE CASCADE,
            step INTEGER NOT NULL,
            edge_key TEXT NOT NULL,
            edge_version INTEGER NOT NULL,
            edge_kind TEXT NOT NULL,
            valid_from REAL NOT NULL,
            valid_to REAL NOT NULL,
            cov_trace REAL NOT NULL,
            UNIQUE(derived_point_id, step)
        );

        CREATE TABLE IF NOT EXISTS event_log (
            id INTEGER PRIMARY KEY,
            at REAL NOT NULL,
            level TEXT NOT NULL,
            code TEXT NOT NULL,
            message TEXT NOT NULL
        );
        "#,
    )?;
    add_column_if_missing(&conn, "generation", "seq_head", "INTEGER")?;
    add_column_if_missing(&conn, "packet", "is_late", "INTEGER NOT NULL DEFAULT 0")?;
    conn.execute(
        "INSERT OR IGNORE INTO meta(key,value) VALUES('schema_version',?1)",
        params![SCHEMA_VERSION.to_string()],
    )?;
    Ok(())
}

impl Db {
    pub fn log_event(&self, level: &str, code: &str, message: &str) {
        if let Ok(c) = self.0.lock() {
            let _ = c.execute(
                "INSERT INTO event_log(at,level,code,message) VALUES(?,?,?,?)",
                params![now_unix(), level, code, message],
            );
        }
    }

    /// 已持锁时写事件（避免重入死锁）。
    pub fn log_event_locked(
        conn: &rusqlite::Connection,
        level: &str,
        code: &str,
        message: &str,
    ) {
        let _ = conn.execute(
            "INSERT INTO event_log(at,level,code,message) VALUES(?,?,?,?)",
            params![now_unix(), level, code, message],
        );
    }

    /// 在阻塞线程池里跑同步 SQLite 操作（rusqlite Connection 非 Send/Sync）。
    pub async fn run<F, T>(&self, f: F) -> anyhow::Result<T>
    where
        F: FnOnce(&Db) -> anyhow::Result<T> + Send + 'static,
        T: Send + 'static,
    {
        let db = self.clone();
        Ok(tokio::task::spawn_blocking(move || f(&db)).await??)
    }

    pub async fn recent_events(&self, limit: i64) -> anyhow::Result<Vec<(i64, f64, String, String, String)>> {
        let db = self.clone();
        tokio::task::spawn_blocking(move || {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let mut stmt = c.prepare(
            "SELECT id,at,level,code,message FROM event_log ORDER BY id DESC LIMIT ?1",
        )?;
        let rows = stmt
            .query_map(params![limit], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    r.get::<_, f64>(1)?,
                    r.get::<_, String>(2)?,
                    r.get::<_, String>(3)?,
                    r.get::<_, String>(4)?,
                ))
            })?
            .collect::<Result<Vec<_>, _>>()?;
        Ok(rows)
        }).await?
    }

    pub fn schema_version(&self) -> anyhow::Result<i64> {
        let c = self.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let v: Option<String> = c
            .query_row("SELECT value FROM meta WHERE key='schema_version'", [], |r| {
                r.get(0)
            })
            .optional()?;
        Ok(v.and_then(|s| s.parse().ok()).unwrap_or(0))
    }
}

pub fn now_unix() -> f64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}
