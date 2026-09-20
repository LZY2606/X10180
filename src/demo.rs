//! 首次启动（空库）时播种一组可核验的演示数据。
//! 场景设备划分：
//! - DEV-01：2027 主采集（迟到包、重叠包、姿态时间缺口），之后“重启”产生新代次（序号归 0、时间倒流）；
//! - DEV-03：1999 年 GPS 10 位周翻转边界；
//! - DEV-04：2016/2017 闰秒附近；
//! - DEV-02：坐标单位毫米（正确标注）。

use crate::geo::Se3;
use crate::ingest::{IncomingPacket, IncomingPose, Ingester};
use crate::time::{utc_to_unix, RawTime};
use crate::db::Db;
use anyhow::Result;

struct Rng(u64);
impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }
}

fn utc_offset(y: i64, m: u32, d: u32, day0: i64, sec_of_day: f64) -> RawTime {
    let total = day0 as f64 * 86400.0 + sec_of_day;
    let day = total as i64 / 86400;
    let rem = total - day as f64 * 86400.0;
    RawTime::Utc {
        y,
        m,
        d: (d as i64 + day) as u32,
        h: (rem as i64 / 3600) as u32,
        min: ((rem as i64 % 3600) / 60) as u32,
        sec: rem % 60.0,
    }
}

fn pose_packet(device: &str, seq: i64, time: RawTime, x: f64, y: f64) -> IncomingPacket {
    let yaw = x * 0.05;
    let q = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
    IncomingPacket {
        device_id: device.into(),
        kind: "pose".into(),
        seq,
        time,
        coord_system: "body".into(),
        unit: "m".into(),
        points: vec![],
        pose: Some(IncomingPose {
            q: [q.i, q.j, q.k, q.w],
            t: [x, y, 0.0],
            cov: Some(serde_json::json!([1e-5,1e-5,1e-5,4e-4,4e-4,4e-4])),
        }),
    }
}

fn lidar_packet(device: &str, seq: i64, time: RawTime, unit: &str) -> IncomingPacket {
    let mut h = 0x9e3779b97f4a7c15_u64;
    for b in device.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h ^= seq as u64;
    let mut rng = Rng(h);
    let mut pts = Vec::new();
    for _ in 0..48 {
        let ang = rng.range(-0.5, 0.5);
        let r = rng.range(2.0, 9.0);
        let v = |x: f64| if unit == "mm" { (x * 1000.0).round() } else { x };
        pts.push(vec![
            v(r * ang.cos()),
            v(r * ang.sin()),
            v(rng.range(-0.4, 0.4)),
            rng.range(0.0, 1.0),
        ]);
    }
    IncomingPacket {
        device_id: device.into(),
        kind: "lidar".into(),
        seq,
        time,
        coord_system: "sensor".into(),
        unit: unit.into(),
        points: pts,
        pose: None,
    }
}

fn add_calib(db: &Db, device: &str, noise: f64, yaw: f64) -> Result<()> {
    crate::graph::add_edge(
        db,
        crate::graph::NewEdge {
            key: format!("calib:{device}:lidar"),
            kind: "calib".into(),
            source_frame: format!("lidar:{device}"),
            target_frame: format!("body:{device}"),
            se3: Se3::from_iso([0.0, 0.0, yaw], [0.0; 3]),
            cov: crate::geo::diag6(noise),
            valid_from: 0.0,
            valid_to: 4.0e9,
        },
    )?;
    Ok(())
}

pub fn seed_if_empty(db: &Db) -> Result<bool> {
    let n: i64 = {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        c.query_row("SELECT COUNT(*) FROM packet", [], |r| r.get(0))?
    };
    if n > 0 {
        return Ok(false);
    }
    seed(db)?;
    Ok(true)
}

pub fn seed(db: &Db) -> Result<()> {
    let ing = Ingester::new();
    let dev = "DEV-01";
    let t = |s: f64| utc_offset(2027, 1, 2, 0, s);

    // ---- 代次 1：50ms 姿态、100ms 点云 ----
    let mut pose_seq = 0i64;
    for i in 0..=10 {
        ing.ingest(db, pose_packet(dev, pose_seq, t(i as f64 * 0.05), i as f64 * 0.1, 0.0))?;
        pose_seq += 1;
    }
    let mut lidar_seq = 0i64;
    for i in 0..5 {
        ing.ingest(db, lidar_packet(dev, lidar_seq, t(i as f64 * 0.1), "m"))?;
        lidar_seq += 1;
    }

    // ---- 迟到包：接收晚到，序号很大，时间落在代次 1 内 ----
    ing.ingest(db, pose_packet(dev, 500, t(0.12), 0.24, 0.0))?;
    // ---- 重叠包：lidar seq=2 的重传，内容一致 ----
    ing.ingest(db, lidar_packet(dev, 2, t(0.2), "m"))?;

    // ---- 姿态时间缺口：最后姿态在 0.5s，下一包从 1.2s 开始（缺口 0.7s） ----
    for i in 12..=22 {
        ing.ingest(db, pose_packet(dev, pose_seq, t(0.6 + i as f64 * 0.05), i as f64 * 0.1, 0.0))?;
        pose_seq += 1;
    }
    // 落在缺口内的 lidar 帧 -> 派生块 error
    ing.ingest(db, lidar_packet(dev, lidar_seq, t(0.8), "m"))?;
    lidar_seq += 1;
    // 缺口恢复后的正常帧（姿态覆盖到 1.7s）
    for k in 0..3 {
        ing.ingest(db, lidar_packet(dev, lidar_seq, t(1.25 + k as f64 * 0.1), "m"))?;
        lidar_seq += 1;
    }

    // ---- DEV-01 设备重启：序号归 0、时间倒流 -> 新代次 ----
    let reboot_day0 = utc_to_unix(2026, 6, 1, 0, 0, 0.0);
    let rt = |s: f64| {
        let total = reboot_day0 + 12.0 * 3600.0 + s;
        let sec = total - utc_to_unix(2026, 6, 1, 0, 0, 0.0);
        RawTime::Utc {
            y: 2026,
            m: 6,
            d: 1,
            h: (sec / 3600.0).floor() as u32,
            min: ((sec % 3600.0) / 60.0).floor() as u32,
            sec: sec % 60.0,
        }
    };
    for i in 0..=6 {
        ing.ingest(db, pose_packet(dev, i, rt(i as f64 * 0.05), 8.0 + i as f64 * 0.1, 1.0))?;
    }
    for i in 0..3 {
        ing.ingest(db, lidar_packet(dev, i, rt(i as f64 * 0.1), "m"))?;
    }

    // ---- DEV-03：GPS 10 位周翻转（1999-08-21/22，周 1023 -> 0） ----
    let devw = "DEV-03";
    for (i, (wk, tow, x)) in [
        (1023i64, 604_799.7_f64, 0.0_f64),
        (1023, 604_799.85, 0.05),
        (0, 0.0, 0.1),
        (0, 0.15, 0.15),
        (0, 0.3, 0.2),
        (0, 0.45, 0.25),
        (0, 0.6, 0.3),
        (0, 0.75, 0.35),
        (0, 0.9, 0.4),
        (0, 1.05, 0.45),
    ].into_iter().enumerate() {
        ing.ingest(db, pose_packet(
            devw,
            i as i64,
            RawTime::GpsWeekTow { week: wk, tow, week_bits: 10 },
            x,
            0.0,
        ))?;
    }
    // 周翻转后继续到来的姿态（与点云夹在同一姿态代次内）。
    ing.ingest(db, lidar_packet(
        devw, 0,
        RawTime::GpsWeekTow { week: 0, tow: 0.67, week_bits: 10 },
        "m",
    ))?;

    // ---- DEV-04：闰秒 2016-12-31 23:59:59.5 / 60.5 / 次日 00:00:00.5 ----
    let devl = "DEV-04";
    for (i, (ts, x)) in [
        // 连续刻度严格单调，相邻间隔 <= 0.2s：
        // 59.6 -> 59.8 -> 60.0(标签60.0=86400.0) -> 60.1(86400.1) -> 次日00:00:00.2
        (RawTime::Utc { y: 2016, m: 12, d: 31, h: 23, min: 59, sec: 59.72 }, 0.0_f64),
        (RawTime::Utc { y: 2016, m: 12, d: 31, h: 23, min: 59, sec: 59.9 }, 0.05),
        (RawTime::Utc { y: 2016, m: 12, d: 31, h: 23, min: 59, sec: 60.08 }, 0.1),
        (RawTime::Utc { y: 2016, m: 12, d: 31, h: 23, min: 59, sec: 60.15 }, 0.15),
        (RawTime::Utc { y: 2017, m: 1, d: 1, h: 0, min: 0, sec: 0.22 }, 0.2),
    ]
    .into_iter()
    .enumerate()
    {

        ing.ingest(db, pose_packet(devl, i as i64, ts, x, 0.0))?;
    }
    ing.ingest(db, lidar_packet(
        devl, 0,
        RawTime::Utc { y: 2016, m: 12, d: 31, h: 23, min: 59, sec: 60.1 },
        "m",
    ))?;

    // ---- DEV-02：毫米单位点云 ----
    let devmm = "DEV-02";
    for i in 0..=4 {
        ing.ingest(db, pose_packet(devmm, i, t(i as f64 * 0.05), 0.0, -3.0))?;
    }
    ing.ingest(db, lidar_packet(devmm, 0, t(0.05), "mm"))?;

    for (d, noise, yaw) in [
        (dev, 5e-4, 0.0),
        (devw, 5e-4, 0.0),
        (devl, 5e-4, 0.0),
        (devmm, 2e-3, 0.02),
    ] {
        add_calib(db, d, noise, yaw)?;
    }

    crate::derive::build_all(db)?;
    Ok(())
}
