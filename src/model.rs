//! 领域模型：原始数据包、变换边、派生块、派生点。

use serde::{Deserialize, Serialize};

/// 数据包类型。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PacketKind {
    Lidar,
    Gnss,
}

impl PacketKind {
    pub fn as_str(self) -> &'static str {
        match self {
            PacketKind::Lidar => "lidar",
            PacketKind::Gnss => "gnss",
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "lidar" => Some(PacketKind::Lidar),
            "gnss" => Some(PacketKind::Gnss),
            _ => None,
        }
    }
}

/// 导入时保存的原始时间戳（不做破坏性改写）。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawStamp {
    pub scale: String,
    pub week: Option<i64>,
    pub sow: Option<f64>,
    pub unix: Option<f64>,
    pub raw_seq: i64,
    pub boot_flag: Option<i64>,
    pub session: Option<String>,
}

/// 单个点的原始记录（导入单位）。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 导入用数据包（外部 JSON）。
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct PacketInput {
    pub kind: PacketKind,
    pub device: String,
    /// lidar / gnss 坐标框名（原始坐标系）。
    pub frame: String,
    /// lidar: meter | millimeter; gnss 平移固定要求 meter。
    pub unit: String,
    pub seq: i64,
    pub boot_flag: Option<i64>,
    pub session: Option<String>,
    pub scale: Option<String>,
    pub week: Option<i64>,
    pub week_bits: Option<u32>,
    pub sow: Option<f64>,
    pub unix: Option<f64>,
    /// lidar: 原始点；gnss 忽略。
    #[serde(default)]
    pub points: Vec<RawPoint>,
    /// gnss 载体位置（米，地图系）。
    #[serde(default)]
    pub tx: f64,
    #[serde(default)]
    pub ty: f64,
    #[serde(default)]
    pub tz: f64,
    /// gnss 姿态四元数 (x,y,z,w)，地图->载体 的旋转。
    #[serde(default)]
    pub qx: f64,
    #[serde(default)]
    pub qy: f64,
    #[serde(default)]
    pub qz: f64,
    #[serde(default)]
    pub qw: f64,
    /// gnss 姿态精度（对角标准差，米/弧度）。
    #[serde(default)]
    pub precision: Option<f64>,
}

/// 入库后的数据包。
#[derive(Debug, Clone, Serialize)]
pub struct Packet {
    pub id: i64,
    pub kind: PacketKind,
    pub device: String,
    pub frame: String,
    pub unit: String,
    pub seq: i64,
    /// 归一化连续 GPS 秒（TAI 时标，代次内单调）。
    pub t: f64,
    pub generation: i64,
    pub duplicate: bool,
    pub late: bool,
    pub raw: RawStamp,
    pub content: String,
}

/// 灵活时间输入：scale + (week,sow|unix|gps)。
#[derive(Debug, Clone, Deserialize, Default)]
pub struct TimeInput {
    pub scale: Option<String>,
    pub week: Option<i64>,
    pub week_bits: Option<u32>,
    pub sow: Option<f64>,
    pub unix: Option<f64>,
    pub gps: Option<f64>,
}

/// 变换边输入：parent <- child（存储 child_to_parent）。
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct TransformInput {
    pub parent: String,
    pub child: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    /// 单位四元数 (x,y,z,w)。
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    /// 6x6 行优先协方差（左扰动 [rho;theta]）。
    #[serde(default)]
    pub covariance: Vec<f64>,
    /// 越小越精确；缺省由协方差迹推断。
    pub precision: Option<f64>,
    pub valid_from: Option<TimeInput>,
    pub valid_to: Option<TimeInput>,
    pub note: Option<String>,
}

/// 变换版本（边）。
#[derive(Debug, Clone, Serialize)]
pub struct Edge {
    pub id: i64,
    pub version: i64,
    pub parent: String,
    pub child: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub covariance_json: String,
    pub precision: f64,
    pub valid_from: f64,
    pub valid_to: f64,
    pub note: Option<String>,
    pub inserted_at: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct DerivedBlock {
    pub id: i64,
    pub generation: i64,
    pub t0: f64,
    pub t1: f64,
    pub status: String,
    pub point_count: i64,
    pub hash: String,
    pub warning: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct DerivedPoint {
    pub id: i64,
    pub block_id: i64,
    pub packet_id: i64,
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub color: String,
    pub sigmas: String,
}
