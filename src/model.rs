//! Plain data types shared across import, storage and the web layer.

use crate::time::RawTime;
use crate::units::UnitSpec;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug)]
pub struct AppError {
    pub code: &'static str,
    pub message: String,
}

impl AppError {
    pub fn new(code: &'static str, message: impl Into<String>) -> Self {
        AppError {
            code,
            message: message.into(),
        }
    }
    pub fn bad_unit(m: impl Into<String>) -> Self {
        AppError::new("unit_error", m)
    }
    pub fn singular(m: impl Into<String>) -> Self {
        AppError::new("singular_covariance", m)
    }
    pub fn cycle(m: impl Into<String>) -> Self {
        AppError::new("cycle_detected", m)
    }
    pub fn not_found(m: impl Into<String>) -> Self {
        AppError::new("not_found", m)
    }
}

impl std::fmt::Display for AppError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}
impl std::error::Error for AppError {}

pub type AppResult<T> = Result<T, AppError>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PacketKind {
    Lidar,
    Gnss,
}

impl PacketKind {
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "lidar" | "laser" | "scan" => Some(PacketKind::Lidar),
            "gnss" | "ins" | "imu_gnss" | "pose" => Some(PacketKind::Gnss),
            _ => None,
        }
    }
}

/// One point inside a lidar packet, in *raw sensor frame* coordinates/units.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RawPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    #[serde(default)]
    pub intensity: Option<f64>,
}

/// Import payload for a single packet.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PacketInput {
    pub kind: String,
    pub device_id: String,
    /// Per-device packet sequence number (resets on reboot).
    pub seq: i64,
    /// Monotonic boot/uptime counter when the device provides one.
    #[serde(default)]
    pub boot_id: Option<i64>,
    /// Free-form boot/session marker ("reboot-A", uuid, ...).
    #[serde(default)]
    pub boot_marker: Option<String>,
    pub timestamp: f64,
    #[serde(default)]
    pub week: Option<i64>,
    #[serde(default)]
    pub leap_second_flag: bool,
    #[serde(default)]
    pub time_scale: Option<String>,
    #[serde(default)]
    pub coord_frame: Option<String>,
    #[serde(default)]
    pub length_unit: Option<String>,
    #[serde(default)]
    pub angle_unit: Option<String>,
    // GNSS pose payload (metres/radians after normalization).
    #[serde(default)]
    pub px: Option<f64>,
    #[serde(default)]
    pub py: Option<f64>,
    #[serde(default)]
    pub pz: Option<f64>,
    #[serde(default)]
    pub roll: Option<f64>,
    #[serde(default)]
    pub pitch: Option<f64>,
    #[serde(default)]
    pub yaw: Option<f64>,
    #[serde(default)]
    pub quat: Option<[f64; 4]>,
    #[serde(default)]
    pub cov: Option<Vec<f64>>,
    // LiDAR payload.
    #[serde(default)]
    pub points: Option<Vec<RawPoint>>,
}

/// Normalized packet record (values converted to SI; raw metadata retained).
#[derive(Clone, Debug, Serialize)]
pub struct PacketRecord {
    pub id: i64,
    pub generation_id: i64,
    pub kind: PacketKind,
    pub device_id: String,
    pub seq: i64,
    pub boot_id: Option<i64>,
    pub boot_marker: Option<String>,
    pub raw: RawTime,
    pub units: UnitSpec,
    pub coord_frame: String,
    pub continuous_time: f64,
    pub received_index: i64,
    pub duplicate_of: Option<i64>,
    pub content_summary: String,
}
