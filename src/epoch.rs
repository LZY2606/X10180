//! 采集代次划分：GPS 周翻转、闰秒、设备重启导致的序号重复都不能只按时间戳排序。
//! 按到达顺序（recv）处理每个设备的包，输出 (epoch, 解缠后的单调时间)。

pub const GPS_WEEK_SECS: f64 = 604_800.0;
/// 闰秒/小抖动容忍：原始时间回退在该范围内视为同一秒级事件，不划代。
pub const LEAP_TOL_SECS: f64 = 3.0;
/// 超过该回退且不能用整周解释时，视为新代次。
pub const WEEK_TOL_SECS: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stamp {
    pub epoch: u32,
    /// 解缠后的单调时间（秒）。
    pub t: f64,
}

/// 单设备代次划分器。按包到达顺序喂入 (seq, t_raw)。
#[derive(Debug, Default)]
pub struct EpochSplitter {
    epoch: u32,
    started: bool,
    prev_seq: u64,
    prev_t: f64,
    /// 解缠偏移（周翻转与闰秒累积）。
    offset: f64,
}

impl EpochSplitter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, seq: u64, t_raw: f64) -> Stamp {
        if !self.started {
            self.started = true;
            self.epoch = 0;
            self.offset = 0.0;
            self.prev_seq = seq;
            self.prev_t = t_raw;
            return Stamp { epoch: self.epoch, t: t_raw };
        }

        let dt_raw = t_raw + self.offset - self.prev_t;

        // 设备重启：序号回退/重复（非时间环绕可解释）→ 新代次。
        if seq <= self.prev_seq && dt_raw < LEAP_TOL_SECS {
            self.start_new_epoch(seq, t_raw);
            return Stamp { epoch: self.epoch, t: self.prev_t };
        }

        if dt_raw < -LEAP_TOL_SECS {
            // 明显回退：先尝试整周翻转解释。
            let weeks = ((-dt_raw) / GPS_WEEK_SECS).round();
            if weeks >= 1.0 && (dt_raw + weeks * GPS_WEEK_SECS).abs() <= WEEK_TOL_SECS {
                self.offset += weeks * GPS_WEEK_SECS;
            } else {
                // 无法解释的回退 → 新代次。
                self.start_new_epoch(seq, t_raw);
                return Stamp { epoch: self.epoch, t: self.prev_t };
            }
        } else if dt_raw < 0.0 {
            // 闰秒级小回退：同代次，累积偏移保持单调。
            self.offset += -dt_raw;
        }

        let t = t_raw + self.offset;
        self.prev_seq = seq;
        self.prev_t = t;
        Stamp { epoch: self.epoch, t }
    }

    fn start_new_epoch(&mut self, seq: u64, t_raw: f64) {
        self.epoch += 1;
        self.offset = 0.0;
        self.prev_seq = seq;
        self.prev_t = t_raw;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_week_rollover_stays_in_epoch() {
        let mut sp = EpochSplitter::new();
        let a = sp.push(1, GPS_WEEK_SECS - 2.0);
        let b = sp.push(2, GPS_WEEK_SECS - 1.0);
        let c = sp.push(3, 1.0); // 周翻转
        assert_eq!(a.epoch, b.epoch);
        assert_eq!(b.epoch, c.epoch);
        assert!((c.t - b.t - 2.0).abs() < 1e-9, "unwrapped t={}", c.t);
    }

    #[test]
    fn duplicate_seq_after_restart_starts_new_epoch() {
        let mut sp = EpochSplitter::new();
        sp.push(10, 100.0);
        sp.push(11, 101.0);
        let s = sp.push(10, 100.0); // 重启后序号重复
        assert_eq!(s.epoch, 1);
    }

    #[test]
    fn leap_second_does_not_split_epoch() {
        let mut sp = EpochSplitter::new();
        let a = sp.push(1, 1000.0);
        let b = sp.push(2, 999.0); // 闰秒回退 1s
        let c = sp.push(3, 1001.0);
        assert_eq!(a.epoch, b.epoch);
        assert_eq!(b.epoch, c.epoch);
        assert!(b.t >= a.t && c.t >= b.t, "monotone: {} {} {}", a.t, b.t, c.t);
    }
}
