//! GPS time handling.
//!
//! Devices report time-of-week seconds (which wrap every GPS week, i.e.
//! `WEEK_SECONDS`), occasionally a week number, and may sit near a UTC leap
//! second.  We never rely on raw timestamps alone for ordering: import first
//! partitions packets into *generations* (continuous acquisition runs), then
//! unwraps time inside a generation into a strictly increasing `t` measured in
//! SI seconds on a continuous GPS timescale.

use serde::{Deserialize, Serialize};

pub const WEEK_SECONDS: f64 = 604_800.0;

/// A raw timestamp exactly as it arrived in a packet.  Nothing here is
/// reordered or "fixed" — the converted continuous time lives separately.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RawTime {
    /// Time-of-week seconds as reported by the device (0..604800).
    pub sow: f64,
    /// GPS week number, if the packet carried one.
    pub week: Option<i64>,
    /// True when the packet was flagged at import as adjacent to a leap second.
    pub leap_second_flag: bool,
    /// Free-form original time scale label ("gps", "utc", "device_uptime").
    pub scale: String,
}

/// Continuous, monotone time in seconds on the unwrapped GPS timescale.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ContinuousTime(pub f64);

impl ContinuousTime {
    pub fn secs(self) -> f64 {
        self.0
    }
}

/// State used while unwrapping time-of-week within a single generation.
#[derive(Clone, Debug)]
pub struct Unwrapper {
    pub week_shift: i64,
    pub prev_continuous: Option<f64>,
}

impl Unwrapper {
    pub fn new() -> Self {
        Unwrapper {
            week_shift: 0,
            prev_continuous: None,
        }
    }


    /// Resolve a raw packet timestamp to continuous seconds.
    ///
    /// `observed_week_shift` is inferred from a packet week number when
    /// present.  A backward jump close to a whole week is treated as a GPS
    /// week rollover (and unwrapped); a jump of about one second combined with
    /// `leap_flag` is a leap-second boundary (continuous time keeps flowing,
    /// the flag is preserved on the packet).  A *large* backward jump without
    /// those explanations means the caller must start a new generation.
    pub fn resolve(&mut self, raw: &RawTime) -> ResolvedTime {
        let mut shift = self.week_shift;
        if let Some(w) = raw.week {
            shift = w;
        }
        let candidate = shift as f64 * WEEK_SECONDS + raw.sow;
        let mut wrapped = false;
        let mut leap = false;

        if let Some(prev) = self.prev_continuous {
            let mut c = candidate;
            let dt = c - prev;
            if dt < -WEEK_SECONDS / 2.0 {
                // rollover forward: week counter was missing, add a week
                shift += 1;
                c += WEEK_SECONDS;
                wrapped = true;
            } else if dt > WEEK_SECONDS / 2.0 {
                shift -= 1;
                c -= WEEK_SECONDS;
                wrapped = true;
            }
            let dt2 = c - prev;
            if raw.leap_second_flag && dt2.abs() < 2.0 && dt2 < 0.0 {
                // Leap second neighbourhood: keep the continuous clock moving
                // forward by clamping to the previous instant; no data is
                // deleted, the flag is retained for audit.
                c = prev;
                leap = true;
            }
            self.week_shift = shift;
            self.prev_continuous = Some(c);
            return ResolvedTime {
                t: ContinuousTime(c),
                week_rollover: wrapped,
                leap_adjusted: leap,
                backward_seconds: dt2,
            };
        }

        self.week_shift = shift;
        self.prev_continuous = Some(candidate);
        ResolvedTime {
            t: ContinuousTime(candidate),
            week_rollover: false,
            leap_adjusted: false,
            backward_seconds: 0.0,
        }
    }
}

impl Default for Unwrapper {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
pub struct ResolvedTime {
    pub t: ContinuousTime,
    pub week_rollover: bool,
    pub leap_adjusted: bool,
    pub backward_seconds: f64,
}

/// Is an unresolvable backward jump large enough to force a new generation?
/// Rollovers (~a week) and leap seconds (~1s) are handled elsewhere.
pub fn is_generation_break(backward_seconds: f64) -> bool {
    backward_seconds < -2.0 && backward_seconds > -WEEK_SECONDS / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(sow: f64) -> RawTime {
        RawTime {
            sow,
            week: None,
            leap_second_flag: false,
            scale: "gps".into(),
        }
    }

    #[test]
    fn unwraps_week_rollover() {
        let mut u = Unwrapper::new();
        let a = u.resolve(&rt(WEEK_SECONDS - 1.0));
        let b = u.resolve(&rt(1.0));
        assert!(b.week_rollover);
        assert!((b.t.secs() - (WEEK_SECONDS + 1.0)).abs() < 1e-9);
        assert!(b.t.secs() > a.t.secs());
    }

    #[test]
    fn leap_second_keeps_continuity() {
        let mut u = Unwrapper::new();
        u.resolve(&rt(100.0));
        let mut raw = rt(99.5);
        raw.leap_second_flag = true;
        let r = u.resolve(&raw);
        assert!(r.leap_adjusted);
        assert!((r.t.secs() - 100.0).abs() < 1e-9);
    }

    #[test]
    fn big_backward_jump_is_generation_break() {
        assert!(is_generation_break(-30.0));
        assert!(!is_generation_break(-0.5));
    }
}
