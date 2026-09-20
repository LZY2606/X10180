//! 原始时间戳规范化。
//!
//! 导入时保留数据包自带的原始时间分量（GPS 周/周内秒、UTC 日历、设备单调钟），
//! 只把换算得到的连续 Unix 秒用于排序与插值。
//! GPS 10/13 位周翻转在 [`unwrap_week`] 中处理；UTC 闰秒（23:59:60.x）在
//! [`utc_to_unix`] 中映射到当日 86400~86401 秒的连续刻度，避免虚假“时间倒流”。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RawTime {
    /// GPS 周 + 周内秒；week_bits 为设备使用的周计数位宽（常见 10）。
    GpsWeekTow {
        week: i64,
        tow: f64,
        #[serde(default = "default_week_bits")]
        week_bits: u8,
    },
    /// UTC 日历时间；sec 允许取 60.x（闰秒标签）。
    Utc {
        y: i64,
        m: u32,
        d: u32,
        h: u32,
        min: u32,
        sec: f64,
    },
    /// 设备单调钟秒（相对历元，仅同设备同代次内可比）。
    Monotonic { sec: f64 },
}

fn default_week_bits() -> u8 {
    10
}

impl RawTime {
    pub fn describe(&self) -> String {
        match self {
            RawTime::GpsWeekTow {
                week,
                tow,
                week_bits,
            } => format!("gps(w={week},tow={tow:.3},{week_bits}b)"),
            RawTime::Utc {
                y,
                m,
                d,
                h,
                min,
                sec,
            } => format!("utc({y:04}-{m:02}-{d:02}T{h:02}:{min:02}:{sec:06.3})"),
            RawTime::Monotonic { sec } => format!("mono({sec:.3})"),
        }
    }
}

/// (该 UTC 瞬间起生效的 GPS-UTC 闰秒数)，GPS 起点 1980-01-06 之后的官方跳变。
const LEAP_TABLE: &[(i64, i64)] = &[
    (362793600, 1),
    (394329600, 2),
    (425865600, 3),
    (489024000, 4),
    (567993600, 5),
    (631152000, 6),
    (662688000, 7),
    (709948800, 8),
    (741484800, 9),
    (773020800, 10),
    (820454400, 11),
    (867715200, 12),
    (915148800, 13),
    (1136073600, 14),
    (1224806400, 15),
    (1341100800, 16),
    (1435708800, 17),
    (1483228800, 18),
];

/// GPS 起点 1980-01-06T00:00:00Z 的 Unix 秒。
pub const GPS_EPOCH_UNIX: i64 = 315964800;

fn leap_seconds_at(unix_utc: i64) -> i64 {
    LEAP_TABLE
        .iter()
        .rev()
        .find(|(eff, _)| unix_utc >= *eff)
        .map(|(_, ls)| *ls)
        .unwrap_or(0)
}

/// 展开 GPS 周计数。返回连续（不回绕）的周数。
///
/// 依据同一设备上一包的原始周与周内秒：当原始周变小且周内秒位于新周开头
/// （旧包位于上一周末尾）时加一个回绕周期；周位宽 10→1024，13→8192。
pub fn unwrap_week(raw_week: i64, prev_raw_week: Option<i64>, prev_tow: Option<f64>, bits: u8) -> i64 {
    let modulus = 1i64 << bits;
    match (prev_raw_week, prev_tow) {
        (Some(prev), Some(prev_tow)) if raw_week < prev => {
            let rolled = raw_week + modulus * (prev.div_euclid(modulus) + 1);
            if rolled > prev || (rolled == prev && prev_tow > 604800.0 - 2.0) {
                rolled
            } else {
                raw_week.max(0)
            }
        }
        _ => raw_week.max(0),
    }
}

/// 连续 GPS 周 + 周内秒 → Unix 秒（UTC 连续刻度，闰秒处迭代收敛）。
pub fn gps_to_unix(week: i64, tow: f64) -> f64 {
    let base = GPS_EPOCH_UNIX as f64 + week as f64 * 604_800.0 + tow;
    let mut leaps = 18i64;
    for _ in 0..6 {
        let next = leap_seconds_at((base - leaps as f64) as i64);
        if next == leaps {
            break;
        }
        leaps = next;
    }
    base - leaps as f64
}

/// 公历日期 → 距 1970-01-01 的天数（Howard Hinnant 算法，支持 1970 前）。
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) as i64 + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe - 719468
}

/// UTC 日历 → 连续 Unix 秒。
///
/// sec 允许 60.x（闰秒标签）：把它放到当日 86400 与 86401 之间，
/// 即 23:59:60.x → 当日末尾 + (x-60)，保证 23:59:59.x < 闰秒，且不与任何
/// 正常整数秒重合，排序得到严格全序（设备重启仍由序号/倒流判定处理）。
pub fn utc_to_unix(y: i64, m: u32, d: u32, h: u32, min: u32, sec: f64) -> f64 {
    let day = days_from_civil(y, m, d) as f64;
    if sec >= 60.0 {
        day * 86_400.0 + 86_400.0 + (sec - 60.0)
    } else {
        day * 86_400.0 + h as f64 * 3600.0 + min as f64 * 60.0 + sec
    }
}

/// 把原始时间戳换算为连续 Unix 秒。week 需先用 [`unwrap_week`] 展开。
pub fn to_unix(raw: &RawTime, unwrapped_week: Option<i64>) -> f64 {
    match raw {
        RawTime::GpsWeekTow { week, tow, .. } => {
            gps_to_unix(unwrapped_week.unwrap_or(*week), *tow)
        }
        RawTime::Utc {
            y,
            m,
            d,
            h,
            min,
            sec,
        } => utc_to_unix(*y, *m, *d, *h, *min, *sec),
        RawTime::Monotonic { sec } => *sec,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_gps_epoch() {
        assert!((gps_to_unix(0, 0.0) - GPS_EPOCH_UNIX as f64).abs() < 1e-6);
        let w1930 = GPS_EPOCH_UNIX as f64 + 1930.0 * 604_800.0;
        assert!((gps_to_unix(1930, 0.0) - (w1930 - 17.0)).abs() < 1.0);
        assert!((gps_to_unix(1930, 604_800.0) - (w1930 + 604_800.0 - 18.0)).abs() < 1.0);
    }

    #[test]
    fn week_rollover_continues_forward() {
        let t0 = gps_to_unix(unwrap_week(1023, None, None, 10), 604_799.5);
        let w1 = unwrap_week(0, Some(1023), Some(604_799.5), 10);
        assert_eq!(w1, 1024);
        let t1 = gps_to_unix(w1, 0.75);
        assert!(t1 > t0);
        assert!((t1 - t0 - 1.25).abs() < 1e-6);
    }

    #[test]
    fn leap_second_orders_correctly() {
        let before = utc_to_unix(2016, 12, 31, 23, 59, 59.5);
        let leap = utc_to_unix(2016, 12, 31, 23, 59, 60.5);
        let after = utc_to_unix(2017, 1, 1, 0, 0, 0.0);
        assert!(before < leap);
        assert!((leap - after - 0.5).abs() < 1e-6);
        // 闰秒标签 60.0 映射到次日零点连续刻度，但 60.x（x>0）在其之后，严格单调。
        assert!(utc_to_unix(2016, 12, 31, 23, 59, 60.1) > after);
    }
}
