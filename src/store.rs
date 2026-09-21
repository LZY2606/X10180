//! SQLite 持久层。
//! 原始包（含原始时间戳、坐标系、单位、内容摘要）永久保存；
//! 派生点仅作为缓存块存放，块写入是事务性的（pending -> complete），
//! 启动时清理 pending 半块，保证崩溃恢复后没有半块被当成成功。

use crate::epoch::EpochSplitter;
use crate::graph::Edge;
use crate::math::{Mat6, Quat, SE3, Vec3};
use rusqlite::{params, Connection};

pub const VALID_UNITS: &[&str] = &["m", "cm", "mm", "deg", "rad"];

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Packet {
    pub id: i64,
    pub device: String,
    pub kind: String, // "lidar" | "pose"
    pub seq: u64,
    pub t_raw: f64,
    pub frame: String,
    pub unit: String,
    pub summary: String,
    pub payload: serde_json::Value,
    pub epoch: u32,
    pub t: f64, // 代次内解缠时间
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct Block {
    pub id: i64,
    pub packet_id: i64,
    pub status: String,
    pub chain: serde_json::Value, // 经过的变换版本链
    pub points: Vec<[f64; 3]>,
    pub t: f64,
}

pub struct Store {
    pub conn: Connection,
}

#[derive(Debug)]
pub enum StoreError {
    Sql(rusqlite::Error),
    BadUnit(String),
    BadInput(String),
}

impl From<rusqlite::Error> for StoreError {
    fn from(e: rusqlite::Error) -> Self {
        StoreError::Sql(e)
    }
}

impl std::fmt::Display for StoreError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StoreError::Sql(e) => write!(f, "数据库错误: {e}"),
            StoreError::BadUnit(u) => write!(f, "未知单位 '{u}'，支持: {}", VALID_UNITS.join(", ")),
            StoreError::BadInput(m) => write!(f, "输入错误: {m}"),
        }
    }
}

pub fn unit_to_meters(unit: &str) -> Result<f64, StoreError> {
    match unit {
        "m" => Ok(1.0),
        "cm" => Ok(0.01),
        "mm" => Ok(0.001),
        other => Err(StoreError::BadUnit(other.to_string())),
    }
}

impl Store {
    pub fn open(path: &str) -> Result<Self, StoreError> {
        let conn = if path == ":memory:" {
            Connection::open_in_memory()?
        } else {
            Connection::open(path)?
        };
        conn.execute_batch(
            "PRAGMA journal_mode=WAL;
             CREATE TABLE IF NOT EXISTS packets(
               id INTEGER PRIMARY KEY AUTOINCREMENT,
               device TEXT NOT NULL, kind TEXT NOT NULL, seq INTEGER NOT NULL,
               t_raw REAL NOT NULL, frame TEXT NOT NULL, unit TEXT NOT NULL,
               summary TEXT NOT NULL, payload TEXT NOT NULL,
               epoch INTEGER NOT NULL, t REAL NOT NULL, recv INTEGER NOT NULL
             );
             CREATE TABLE IF NOT EXISTS transforms(
               id INTEGER PRIMARY KEY AUTOINCREMENT,
               src TEXT NOT NULL, dst TEXT NOT NULL, version INTEGER NOT NULL,
               rot TEXT NOT NULL, trans TEXT NOT NULL, cov TEXT NOT NULL,
               valid_from REAL NOT NULL, valid_to REAL NOT NULL
             );
             CREATE TABLE IF NOT EXISTS blocks(
               id INTEGER PRIMARY KEY AUTOINCREMENT,
               packet_id INTEGER NOT NULL UNIQUE,
               status TEXT NOT NULL,          -- pending | complete
               chain TEXT NOT NULL,
               points TEXT NOT NULL,
               t REAL NOT NULL
             );",
        )?;
        let mut s = Store { conn };
        s.recover()?;
        Ok(s)
    }

    /// 崩溃恢复：删除所有 pending 半块（它们绝不应被当成成功）。
    pub fn recover(&mut self) -> Result<usize, StoreError> {
        let n = self.conn.execute("DELETE FROM blocks WHERE status='pending'", [])?;
        Ok(n)
    }

    /// 导入数据包：保存原始时间戳/坐标系/单位/摘要，并做代次划分。
    pub fn ingest(
        &mut self,
        device: &str,
        kind: &str,
        seq: u64,
        t_raw: f64,
        frame: &str,
        unit: &str,
        summary: &str,
        payload: serde_json::Value,
    ) -> Result<Packet, StoreError> {
        if !VALID_UNITS.contains(&unit) {
            return Err(StoreError::BadUnit(unit.to_string()));
        }
        if kind != "lidar" && kind != "pose" {
            return Err(StoreError::BadInput(format!("未知包类型 '{kind}'")));
        }
        // 代次划分：重放该设备已有包的到达序列，再喂入新包。
        let mut sp = EpochSplitter::new();
        {
            let mut stmt = self
                .conn
                .prepare("SELECT seq, t_raw FROM packets WHERE device=?1 ORDER BY recv")?;
            let rows = stmt
                .query_map(params![device], |r| Ok((r.get::<_, i64>(0)? as u64, r.get::<_, f64>(1)?)))?
                .collect::<Result<Vec<_>, _>>()?;
            for (s, t) in rows {
                sp.push(s, t);
            }
        }
        let stamp = sp.push(seq, t_raw);
        let recv: i64 = self
            .conn
            .query_row("SELECT COALESCE(MAX(recv),0)+1 FROM packets", [], |r| r.get(0))?;
        self.conn.execute(
            "INSERT INTO packets(device,kind,seq,t_raw,frame,unit,summary,payload,epoch,t,recv)
             VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11)",
            params![
                device, kind, seq as i64, t_raw, frame, unit, summary,
                payload.to_string(), stamp.epoch as i64, stamp.t, recv
            ],
        )?;
        Ok(Packet {
            id: self.conn.last_insert_rowid(),
            device: device.into(),
            kind: kind.into(),
            seq,
            t_raw,
            frame: frame.into(),
            unit: unit.into(),
            summary: summary.into(),
            payload,
            epoch: stamp.epoch,
            t: stamp.t,
        })
    }

    pub fn packets(&self, kind: Option<&str>) -> Result<Vec<Packet>, StoreError> {
        let (sql, p): (String, Vec<String>) = match kind {
            Some(k) => (
                "SELECT id,device,kind,seq,t_raw,frame,unit,summary,payload,epoch,t FROM packets WHERE kind=?1 ORDER BY recv".into(),
                vec![k.to_string()],
            ),
            None => (
                "SELECT id,device,kind,seq,t_raw,frame,unit,summary,payload,epoch,t FROM packets ORDER BY recv".into(),
                vec![],
            ),
        };
        let mut stmt = self.conn.prepare(&sql)?;
        let rows = stmt
            .query_map(rusqlite::params_from_iter(p.iter()), |r| {
                Ok(Packet {
                    id: r.get(0)?,
                    device: r.get(1)?,
                    kind: r.get(2)?,
                    seq: r.get::<_, i64>(3)? as u64,
                    t_raw: r.get(4)?,
                    frame: r.get(5)?,
                    unit: r.get(6)?,
                    summary: r.get(7)?,
                    payload: serde_json::from_str(&r.get::<_, String>(8)?).unwrap_or_default(),
                    epoch: r.get::<_, i64>(9)? as u32,
                    t: r.get(10)?,
                })
            })?
            .collect::<Result<Vec<_>, _>>()?;
        Ok(rows)
    }

    /// 新增变换版本。协方差必须半正定（允许奇异）。
    pub fn add_transform(
        &mut self,
        src: &str,
        dst: &str,
        tf: SE3,
        cov: Mat6,
        valid_from: f64,
        valid_to: f64,
    ) -> Result<Edge, StoreError> {
        if !cov.is_psd(crate::math::PSD_TOL) {
            return Err(StoreError::BadInput("协方差矩阵非半正定".into()));
        }
        if valid_from >= valid_to {
            return Err(StoreError::BadInput("有效时间区间为空".into()));
        }
        let version: i64 = self.conn.query_row(
            "SELECT COALESCE(MAX(version),0)+1 FROM transforms WHERE src=?1 AND dst=?2",
            params![src, dst],
            |r| r.get(0),
        )?;
        self.conn.execute(
            "INSERT INTO transforms(src,dst,version,rot,trans,cov,valid_from,valid_to)
             VALUES(?1,?2,?3,?4,?5,?6,?7,?8)",
            params![
                src, dst, version,
                serde_json::to_string(&tf.rot).unwrap(),
                serde_json::to_string(&tf.trans).unwrap(),
                serde_json::to_string(&cov).unwrap(),
                valid_from, valid_to
            ],
        )?;
        Ok(Edge { src: src.into(), dst: dst.into(), version, tf, cov, valid_from, valid_to })
    }

    pub fn transforms(&self) -> Result<Vec<Edge>, StoreError> {
        let mut stmt = self.conn.prepare(
            "SELECT src,dst,version,rot,trans,cov,valid_from,valid_to FROM transforms ORDER BY src,dst,version",
        )?;
        let rows = stmt
            .query_map([], |r| {
                Ok(Edge {
                    src: r.get(0)?,
                    dst: r.get(1)?,
                    version: r.get(2)?,
                    tf: SE3::new(
                        serde_json::from_str::<Quat>(&r.get::<_, String>(3)?).unwrap(),
                        serde_json::from_str::<Vec3>(&r.get::<_, String>(4)?).unwrap(),
                    ),
                    cov: serde_json::from_str(&r.get::<_, String>(5)?).unwrap(),
                    valid_from: r.get(6)?,
                    valid_to: r.get(7)?,
                })
            })?
            .collect::<Result<Vec<_>, _>>()?;
        Ok(rows)
    }

    /// 标定修订只使覆盖时间段内的派生块失效：删除时间落在 [from,to]
    /// 且链上用到该 (src,dst) 边的 complete 块。返回失效块数。
    pub fn invalidate_blocks(&self, src: &str, dst: &str, from: f64, to: f64) -> Result<usize, StoreError> {
        let needle = format!("\"{}->{}\"", src, dst);
        let n = self.conn.execute(
            "DELETE FROM blocks WHERE status='complete' AND t>=?1 AND t<=?2 AND chain LIKE ?3",
            params![from, to, format!("%{needle}%")],
        )?;
        Ok(n)
    }

    /// 事务性写块：先 pending，再更新为 complete。任一失败整体回滚。
    pub fn write_block(
        &mut self,
        packet_id: i64,
        t: f64,
        chain: &serde_json::Value,
        points: &[[f64; 3]],
    ) -> Result<(), StoreError> {
        let tx = self.conn.transaction()?;
        tx.execute("DELETE FROM blocks WHERE packet_id=?1", params![packet_id])?;
        tx.execute(
            "INSERT INTO blocks(packet_id,status,chain,points,t) VALUES(?1,'pending','[]','[]',?2)",
            params![packet_id, t],
        )?;
        tx.execute(
            "UPDATE blocks SET status='complete', chain=?1, points=?2 WHERE packet_id=?3 AND status='pending'",
            params![chain.to_string(), serde_json::to_string(points).unwrap(), packet_id],
        )?;
        tx.commit()?;
        Ok(())
    }

    pub fn complete_block_packet_ids(&self) -> Result<std::collections::BTreeSet<i64>, StoreError> {
        let mut stmt = self.conn.prepare("SELECT packet_id FROM blocks WHERE status='complete'")?;
        let rows = stmt.query_map([], |r| r.get::<_, i64>(0))?.collect::<Result<Vec<_>, _>>()?;
        Ok(rows.into_iter().collect())
    }

    pub fn blocks(&self) -> Result<Vec<Block>, StoreError> {
        let mut stmt = self
            .conn
            .prepare("SELECT id,packet_id,status,chain,points,t FROM blocks WHERE status='complete' ORDER BY t")?;
        let rows = stmt
            .query_map([], |r| {
                Ok(Block {
                    id: r.get(0)?,
                    packet_id: r.get(1)?,
                    status: r.get(2)?,
                    chain: serde_json::from_str(&r.get::<_, String>(3)?).unwrap_or_default(),
                    points: serde_json::from_str(&r.get::<_, String>(4)?).unwrap_or_default(),
                    t: r.get(5)?,
                })
            })?
            .collect::<Result<Vec<_>, _>>()?;
        Ok(rows)
    }

    pub fn block_by_id(&self, id: i64) -> Result<Option<Block>, StoreError> {
        Ok(self.blocks()?.into_iter().find(|b| b.id == id))
    }
}
