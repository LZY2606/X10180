//! Time handling robust against GPS week rollover, leap seconds, device
//! restarts and reused sequence numbers.
//!
//! All internal time is `gps_ms`: continuous milliseconds since the GPS
//! epoch 1980-01-06T00:00:00Z in *GPS* time (no leap seconds). Imported
//! packets may be expressed as GPS week/sec (subject to 1024-week
//! rollover) or as Unix milliseconds (subject to leap-second smearing or
//! a duplicated leap second). Normalization always requires an anchor of
//! already-seen packets for the same stream: a lone packet cannot be
//! unwrapped.

/// Reference epoch for unwrapping: GPS week 2000 start (2018-07-25).
pub const REF_GPS_WEEK: i64 = 2000;
/// A GPS week in milliseconds.
pub const WEEK_MS: i64 = 7 * 24 * 60 * 60 * 1000;
/// Maximum forward jump inside one collection generation (2 seconds);
/// larger gaps split generations even without an explicit restart flag.
pub const GENERATION_GAP_MS: i64 = 2_000;

/// (Unix day index since 1970-01-01, GPS-UTC offset after that day's
/// leap second event). Sorted. Covers 1980..2017; offsets after the last
/// table entry remain 18 s.
pub const LEAP_TABLE: &[(i64, i64)] = &[
    // Unix day (1970 epoch) on which the new GPS-UTC offset takes effect
    // (i.e. the UTC calendar day *after* each positive leap second).
    (4199, 1),    // 1981-06-30
    (4564, 2),    // 1982-06-30
    (4929, 3),    // 1983-06-30
    (5660, 4),    // 1985-06-30
    (6574, 5),    // 1987-12-31
    (7305, 6),    // 1989-12-31
    (7670, 7),    // 1990-12-31
    (8217, 8),    // 1992-06-30
    (8582, 9),    // 1993-06-30
    (8947, 10),   // 1994-06-30
    (9496, 11),   // 1995-12-31
    (10043, 12),  // 1997-06-30
    (10592, 13),  // 1998-12-31
    (13149, 14),  // 2005-12-31
    (14245, 15),  // 2008-12-31
    (15522, 16),  // 2012-06-30
    (16617, 17),  // 2015-06-30
    (17167, 18),  // 2016-12-31 -> 2017-01-01
];

/// GPS epoch as days since Unix epoch: 1980-01-06.
pub const GPS_EPOCH_DAY: i64 = 3657;

fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    // Howard Hinnant's algorithm.
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) as i64 + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe - 719468
}

/// Milliseconds since GPS epoch for a UTC wall-clock date/time, with the
/// current leap-second offset applied. Used mainly in tests/demo.
pub fn utc_ymd_hms_ms(y: i64, m: u32, d: u32, h: u32, min: u32, s: u32, ms: u32) -> i64 {
    // Build the Unix millisecond of the stated wall clock and convert via
    // the leap-second-aware path so the repeated final second is handled.
    let unix_day = days_from_civil(y, m, d);
    let unix_ms = unix_day * 86_400_000
        + h as i64 * 3_600_000
        + min as i64 * 60_000
        + s as i64 * 1_000
        + ms as i64;
    unix_ms_to_gps_ms(unix_ms, false)
}

fn offset_after_day(unix_day: i64) -> i64 {
    let mut off = 0;
    for (d, val) in LEAP_TABLE {
        if unix_day >= *d {
            off = *val;
        }
    }
    off
}

/// GPS-UTC offset (seconds) for a Unix millisecond. On the day a positive
/// leap second is inserted, the last UTC second before midnight is
/// reported twice by many receivers: `leap_second_occurred` selects the
/// second occurrence, which already runs at the post-leap GPS offset.
fn leap_offset_at_unix_ms(unix_ms: i64, leap_second_occurred: bool) -> i64 {
    let day = unix_ms.div_euclid(86_400_000);
    let tod = unix_ms.rem_euclid(86_400_000);
    let next_offset = LEAP_TABLE
        .iter()
        .find(|(d, _)| *d == day + 1)
        .map(|(_, v)| *v);
    if let Some(new_offset) = next_offset {
        if tod >= 86_399_000 {
            // First occurrence of the repeated UTC second still runs at
            // the old offset; the leap second itself takes the new one.
            return if leap_second_occurred { new_offset } else { new_offset - 1 };
        }
    }
    offset_after_day(day)
}


/// Convert Unix milliseconds to continuous GPS milliseconds.
/// `leap_second_occurred=true` selects the second 60 (the duplicated
/// positive leap second, Unix timestamps repeat that second on many
/// receivers); it adds one extra second so the two readings separate.
pub fn unix_ms_to_gps_ms(unix_ms: i64, leap_second_occurred: bool) -> i64 {
    let off = leap_offset_at_unix_ms(unix_ms, leap_second_occurred) * 1_000;
    unix_ms - GPS_EPOCH_DAY * 86_400_000 + off
}

/// Continuous GPS milliseconds for a GPS week / ms-of-week reading where
/// the week number has rolled over to 10-bit (0..=1023). `anchor_ms` is a
/// previously normalized time from the same stream; the reading is placed
/// in the rollover epoch closest to the anchor (never more than half a
/// rollover period away).
pub fn unwrap_gps_week(week10: u32, ms_of_week: f64, anchor_ms: i64) -> i64 {
    let local = (week10 as i64) * WEEK_MS + ms_of_week as i64;
    let rollover = 1024 * WEEK_MS;
    let anchor_epoch = anchor_ms.div_euclid(rollover);
    let mut best = anchor_epoch * rollover + local;
    let mut best_dist = (best - anchor_ms).abs();
    for cand in [best - rollover, best + rollover] {
        let dist = (cand - anchor_ms).abs();
        if dist < best_dist {
            best = cand;
            best_dist = dist;
        }
    }
    best
}

/// First packet of a stream: choose the rollover epoch containing
/// [`REF_GPS_WEEK`].
pub fn unwrap_gps_week_first(week10: u32, ms_of_week: f64) -> i64 {
    let local = (week10 as i64) * WEEK_MS + ms_of_week as i64;
    let rollover = 1024 * WEEK_MS;
    let ref_start = REF_GPS_WEEK * WEEK_MS;
    let epoch = ref_start.div_euclid(rollover);
    let cand = epoch * rollover + local;
    if cand >= ref_start + rollover {
        cand - rollover
    } else if cand < ref_start {
        cand + rollover
    } else {
        cand
    }
}

/// Classification emitted by [`assign_generations`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketFlag {
    Normal,
    /// Sequence number restarted near zero / explicit device restart.
    GenerationRestart,
    /// Forward time jump larger than [`GENERATION_GAP_MS`].
    GenerationGap,
    /// Timestamp arrived earlier than the newest packet of its generation
    /// but inside the same generation (a late packet).
    Late,
    /// Same (generation, sequence) seen before with identical content.
    Duplicate,
    /// Same (generation, sequence) seen before with different content.
    Overlap,
}

pub struct AssignedPacket {
    pub index: usize,
    pub generation: u32,
    pub flag: PacketFlag,
}

/// Input packet in receive order.
pub struct RawPacket {
    /// Normalized continuous GPS time.
    pub time_ms: i64,
    /// Device sequence number (wraps at the device's modulus, e.g. 65536).
    pub seq: u32,
    pub seq_modulus: u32,
    pub restart_flag: bool,
    /// FNV-style content hash; identical content plus identical key means
    /// a pure duplicate rather than an overlap conflict.
    pub content_hash: u64,
}

/// Split a packet stream into collection generations using only
/// deterministic, order-independent rules. Packets are processed in
/// receive order; generation assignment never depends on sorting by time.
pub fn assign_generations(packets: &[RawPacket]) -> Vec<AssignedPacket> {
    let mut out: Vec<AssignedPacket> = Vec::with_capacity(packets.len());
    let mut gen: u32 = 0;
    // Keyed by (generation, seq) -> content hash.
    let mut seen: Vec<((u32, u32), u64)> = Vec::new();
    let mut prev_seq: Option<u32> = None;
    let mut newest_time = i64::MIN;
    let mut gen_start_time: Option<i64> = None;

    for (index, p) in packets.iter().enumerate() {
        let mut flag = PacketFlag::Normal;
        let mut new_gen = false;

        if index == 0 {
            new_gen = true;
        } else if p.restart_flag {
            new_gen = true;
            flag = PacketFlag::GenerationRestart;
        } else if let Some(prev) = prev_seq {
            // A counter that jumps backwards to a small value means a
            // device restart / counter reset. The only legal backwards
            // jump to zero is the natural modulus wrap modulus-1 -> 0.
            let normal_wrap = prev == p.seq_modulus - 1 && p.seq == 0;
            // A reset is a big backwards jump relative to the counter
            // range, or an unexplained return to exactly zero.
            let big_backwards = (p.seq as i64) < (prev as i64) - (p.seq_modulus as i64 / 8);
            let return_to_zero = p.seq == 0 && prev != 0 && !normal_wrap;
            if !normal_wrap && (big_backwards || return_to_zero) {
                new_gen = true;
                flag = PacketFlag::GenerationRestart;
            }
        }

        if !new_gen {
            if let Some(start) = gen_start_time {
                if p.time_ms - newest_time > GENERATION_GAP_MS
                    || p.time_ms < start - GENERATION_GAP_MS
                {
                    new_gen = true;
                    if flag == PacketFlag::Normal {
                        flag = PacketFlag::GenerationGap;
                    }
                }
            }
        }

        if new_gen {
            gen += 1;
            seen.clear();
            newest_time = p.time_ms;
            gen_start_time = Some(p.time_ms);
        } else if p.time_ms < newest_time {
            // Late packet within the same generation.
            flag = PacketFlag::Late;
        }

        // Duplicate / overlap detection within the generation.
        let key = (gen, p.seq);
        if let Some((_, hash)) = seen.iter().find(|(k, _)| *k == key).copied() {
            if hash == p.content_hash {
                flag = PacketFlag::Duplicate;
            } else if flag == PacketFlag::Normal || flag == PacketFlag::Late {
                flag = PacketFlag::Overlap;
            }
        } else {
            seen.push((key, p.content_hash));
        }

        if p.time_ms > newest_time {
            newest_time = p.time_ms;
        }
        prev_seq = Some(p.seq);
        out.push(AssignedPacket { index, generation: gen, flag });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leap_second_2017_separates_repeated_second() {
        // 2017-01-01 00:00:00 UTC: GPS is 18 seconds ahead.
        let g0 = utc_ymd_hms_ms(2016, 12, 31, 23, 59, 59, 0);
        let g1 = utc_ymd_hms_ms(2017, 1, 1, 0, 0, 0, 0);
        // A positive leap second sits between these two UTC wall-clock
        // labels, so continuous GPS time advances by 2 seconds.
        assert_eq!(g1 - g0, 2_000);
        // Unix times of the smeared leap second: 23:59:60 shares the
        // integer Unix second 1483228799 with 23:59:59.
        let unix_dup = 1_483_228_799_000i64;
        let a = unix_ms_to_gps_ms(unix_dup, false);
        let b = unix_ms_to_gps_ms(unix_dup, true);
        assert_eq!(b - a, 1_000);
        // Sanity on the offset.
        let gps_epoch = unix_ms_to_gps_ms(0, false);
        assert!(gps_epoch < 0);
        // Midnight after the leap uses the new 18 s offset.
        assert_eq!(unix_ms_to_gps_ms(1_483_228_800_000, false),
                   1_483_228_800_000 - GPS_EPOCH_DAY * 86_400_000 + 18_000);
    }

    #[test]
    fn gps_week_rollover_unwraps_near_anchor() {
        // Week 2046, 12.345 s into the week: far past the 10-bit range.
        let full_week: i64 = 2046;
        let msow = 12_345.0;
        let truth = full_week * WEEK_MS + msow as i64;
        let week10 = (full_week % 1024) as u32; // 1022
        // First-seen anchoring to reference epoch (week 2000..3024).
        let first = unwrap_gps_week_first(week10, msow);
        assert_eq!(first, truth, "first = {}, truth = {}", first, truth);
        // Subsequent readings snap to the nearest rollover epoch.
        assert_eq!(unwrap_gps_week(week10, msow, truth), truth);
        let next = unwrap_gps_week(week10, msow + 500.0, truth);
        assert_eq!(next, truth + 500);
    }

    fn pkt(time_ms: i64, seq: u32, hash: u64) -> RawPacket {
        RawPacket { time_ms, seq, seq_modulus: 65536, restart_flag: false, content_hash: hash }
    }

    #[test]
    fn restart_flag_starts_generation() {
        let ps = vec![pkt(1000, 0, 1), pkt(1100, 1, 2)];
        let mut ps = ps;
        ps.push(RawPacket { time_ms: 1200, seq: 0, seq_modulus: 65536, restart_flag: true, content_hash: 3 });
        let a = assign_generations(&ps);
        assert_eq!(a[0].generation, 1);
        assert_eq!(a[1].generation, 1);
        assert_eq!(a[2].generation, 2);
        assert_eq!(a[2].flag, PacketFlag::GenerationRestart);
    }

    #[test]
    fn normal_u16_wrap_does_not_split_generation() {
        let ps = vec![pkt(1000, 65535, 1), pkt(1100, 0, 2), pkt(1200, 1, 3)];
        let a = assign_generations(&ps);
        assert_eq!(a[0].generation, a[1].generation);
        assert_eq!(a[1].generation, a[2].generation);
        assert_eq!(a[1].flag, PacketFlag::Normal);
    }

    #[test]
    fn reused_seq_after_normal_progress_restarts() {
        let ps = vec![pkt(1000, 100, 1), pkt(1100, 101, 2), pkt(1200, 0, 3)];
        let a = assign_generations(&ps);
        assert_eq!(a[2].generation, 2);
        assert_eq!(a[2].flag, PacketFlag::GenerationRestart);
    }

    #[test]
    fn time_gap_starts_generation_but_small_gap_does_not() {
        let small = vec![pkt(1000, 0, 1), pkt(1500, 1, 2)];
        assert_eq!(assign_generations(&small)[1].generation, 1);
        let big = vec![pkt(1000, 0, 1), pkt(1000 + GENERATION_GAP_MS + 1, 1, 2)];
        let a = assign_generations(&big);
        assert_eq!(a[1].generation, 2);
        assert_eq!(a[1].flag, PacketFlag::GenerationGap);
    }

    #[test]
    fn late_packet_is_flagged_within_generation() {
        let ps = vec![pkt(1000, 0, 1), pkt(1300, 2, 3), pkt(1100, 1, 2)];
        let a = assign_generations(&ps);
        assert_eq!(a[2].generation, 1);
        assert_eq!(a[2].flag, PacketFlag::Late);
    }

    #[test]
    fn duplicate_and_overlap_detection() {
        let same = vec![pkt(1000, 5, 9), pkt(1100, 5, 9)];
        assert_eq!(assign_generations(&same)[1].flag, PacketFlag::Duplicate);
        let conflict = vec![pkt(1000, 5, 9), pkt(1100, 5, 10)];
        assert_eq!(assign_generations(&conflict)[1].flag, PacketFlag::Overlap);
    }

    #[test]
    fn seq_reuse_after_gap_is_generation_not_duplicate() {
        // Time gap forces a new generation first; seq reuse there must
        // not be misread as a duplicate of gen 1.
        let ps = vec![
            pkt(1000, 5, 9),
            pkt(1000 + GENERATION_GAP_MS + 10, 5, 9),
        ];
        let a = assign_generations(&ps);
        assert_eq!(a[1].generation, 2);
        assert_ne!(a[1].flag, PacketFlag::Duplicate);
    }
}
