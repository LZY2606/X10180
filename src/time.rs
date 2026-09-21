//! 时间归一化：UTC/GPS 时标、GPS 周翻转、闰秒。
//! 内部统一使用连续 GPS 秒（TAI 时标，不含闰秒）。

/// (UTC 民用日期, 该日起 TAI-UTC 秒数)。
const LEAP_TABLE: &[(i64, u32, u32, i64)] = &[
    (1980, 1, 1, 19),
    (1981, 7, 1, 20),
    (1982, 7, 1, 21),
    (1983, 7, 1, 22),
    (1985, 7, 1, 23),
    (1988, 1, 1, 24),
    (1990, 1, 1, 25),
    (1991, 1, 1, 26),
    (1992, 7, 1, 27),
    (1993, 7, 1, 28),
    (1994, 7, 1, 29),
    (1996, 1, 1, 30),
    (1997, 7, 1, 31),
    (1999, 1, 1, 32),
    (2006, 1, 1, 33),
    (2009, 1, 1, 34),
    (2012, 7, 1, 35),
    (2015, 7, 1, 36),
    (2017, 1, 1, 37),
];

/// 表后日期的 TAI-UTC（截至 2026 年仍为 37）。
pub fn current_tai_offset() -> i64 {
    37
}

/// GPS-UTC = TAI-UTC - 19。
pub fn leap_seconds_at(unix_seconds: f64) -> i64 {
    let day = (unix_seconds / 86400.0).floor() as i64;
    let entry = LEAP_TABLE
        .iter()
        .rev()
        .find(|(y, m, d)| days_from_civil(*y, *m, *d) <= day);
    match entry {
        Some((_, _, _, tai)) => tai - 19,
        None => 0,
    }
}

/// Unix UTC 秒 -> 连续 GPS 秒（1980-01-06T00:00:00Z 起）。
pub fn unix_to_gps(unix_seconds: f64) -> f64 {
    unix_seconds - 315_964_800.0 + leap_seconds_at(unix_seconds) as f64
}

/// GPS 周 + 周内秒 -> 连续 GPS 秒。
pub fn week_sow_to_gps(week: i64, sow: f64) -> f64 {
    week as f64 * 604_800.0 + sow
}

/// 给定连续 GPS 秒所属的完整 GPS 周。
pub fn gps_week(gps_seconds: f64) -> i64 {
    (gps_seconds / 604_800.0).floor() as i64
}

/// 解析可能截断的 GPS 周（典型 10 bit 回绕），选距参考周最近的完整周。
pub fn unwrap_week(raw_week: i64, week_bits: Option<u32>, ref_week: i64) -> i64 {
    match week_bits {
        None | Some(0) => raw_week,
        Some(bits) => {
            let modulus = 1i64 << bits;
            let r = raw_week.rem_euclid(modulus);
            let base = ref_week - ref_week.rem_euclid(modulus);
            let mut best = base + r;
            let mut best_dist = (best - ref_week).abs();
            for k in [-2i64, -1, 1, 2] {
                let cand = best + k * modulus;
                let dist = (cand - ref_week).abs();
                if dist < best_dist {
                    best_dist = dist;
                    best = cand;
                }
            }
            best
        }
    }
}

/// 原始时间字段归一化为连续 GPS 秒。
pub fn normalize(
    scale: Option<&str>,
    week: Option<i64>,
    week_bits: Option<u32>,
    sow: Option<f64>,
    unix: Option<f64>,
    ref_week: i64,
) -> Result<f64, String> {
    let scale = scale.unwrap_or("gps").to_ascii_lowercase();
    match scale.as_str() {
        "gps" | "gpst" => {
            let s = sow.ok_or_else(|| "gps 时标需要 sow".to_string())?;
            let w = match week {
                Some(w) => unwrap_week(w, week_bits, ref_week),
                None => {
                    // 只有 sow 时按周内秒落在哪一周不明确，使用参考周；
                    // 周翻转由显式 week/week_bits 路径测试覆盖。
                    ref_week
                }
            };
            Ok(week_sow_to_gps(w, s))
        }
        "unix" | "utc" => {
            let u = unix
                .or(sow)
                .ok_or_else(|| "utc/unix 时标需要 unix 秒".to_string())?;
            Ok(unix_to_gps(u))
        }
        other => Err(format!("未知时间尺度: {other}")),
    }
}

/// TimeInput 风格的灵活解析（gps 直通，已是连续 GPS 秒）。
pub fn normalize_flex(
    scale: Option<&str>,
    week: Option<i64>,
    week_bits: Option<u32>,
    sow: Option<f64>,
    unix: Option<f64>,
    gps: Option<f64>,
    ref_week: i64,
) -> Result<f64, String> {
    if let Some(g) = gps {
        return Ok(g);
    }
    normalize(scale, week, week_bits, sow, unix, ref_week)
}

/// Howard Hinnant 天数 -> 公历。
pub fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z2 = z + 719468;
    let era = if z2 >= 0 { z2 } else { z2 - 146096 } / 146097;
    let doe = z2 - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = (if mp < 10 { mp + 3 } else { mp - 9 }) as u32;
    (y + i64::from(m <= 2), m, d)
}

/// 公历 -> 自 1970-01-01 天数。
pub fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y2 = y - i64::from(m <= 2);
    let era = if y2 >= 0 { y2 } else { y2 - 399 } / 400;
    let yoe = y2 - era * 400;
    let doy = (153 * (if m > 2 { i64::from(m) - 3 } else { i64::from(m) + 9 }) + 2) / 5
        + i64::from(d)
        - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe - 719468
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leap_seconds_known_epochs() {
        assert_eq!(leap_seconds_at(946684800.0), 13); // 2000-01-01
        assert_eq!(leap_seconds_at(1_435_708_800.0), 17); // 2015-07-01
        assert_eq!(leap_seconds_at(1_483_228_800.0), 18); // 2017-01-01
        assert_eq!(leap_seconds_at(915148800.0), 13); // 1999-01-01
        assert_eq!(leap_seconds_at(315_964_800.0), 0); // GPS 纪元
    }

    #[test]
    fn gps_epoch_anchor() {
        assert!((unix_to_gps(315_964_800.0)).abs() < 1e-9);
        // 2017 后 unix 与 gps 差 315964818。
        assert!((unix_to_gps(1_700_000_000.0) - (1_700_000_000.0 - 315_964_818.0)).abs() < 1e-6);
    }

    #[test]
    fn week_rollover_10bit() {
        // 2019-04-08 附近为 GPS 周 2048，10bit 回绕后显示 0。
        let g = unix_to_gps(1_554_681_600.0);
        let rw = gps_week(g);
        assert_eq!(rw, 2048);
        assert_eq!(unwrap_week(0, Some(10), rw), 2048);
        assert_eq!(unwrap_week(2047, Some(10), rw), 2047);
        // 2026 年附近周号 ~2433，10bit 值 ~385。
        let g2 = unix_to_gps(1_789_000_000.0);
        let rw2 = gps_week(g2);
        assert_eq!(unwrap_week(rw2.rem_euclid(1024), Some(10), rw2), rw2);
    }

    #[test]
    fn civil_date_roundtrip() {
        for (y, m, d) in [(1970, 1u32, 1u32), (2017, 1, 1), (1999, 12, 31), (2026, 9, 22)] {
            let days = days_from_civil(y, m, d);
            assert_eq!(civil_from_days(days), (y, m, d));
        }
    }

    #[test]
    fn unknown_scale_errors() {
        assert!(normalize(Some("galileo"), None, None, Some(1.0), None, 2000).is_err());
    }
}
