//! Unit normalisation.
//!
//! Internally everything is stored in SI (metres, radians).  Raw packets keep
//! a record of the units they arrived with; only derived caches use SI.  An
//! unknown or incompatible unit is a hard import error — silent guessing is
//! exactly how a point cloud gets scaled by 1000.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LengthUnit {
    Meter,
    Millimeter,
    Centimeter,
    Foot,
    Inch,
}

impl LengthUnit {
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "m" | "meter" | "meters" | "metre" | "metres" => Some(LengthUnit::Meter),
            "mm" | "millimeter" | "millimeters" | "millimetre" => Some(LengthUnit::Millimeter),
            "cm" | "centimeter" | "centimeters" | "centimetre" => Some(LengthUnit::Centimeter),
            "ft" | "foot" | "feet" => Some(LengthUnit::Foot),
            "in" | "inch" | "inches" => Some(LengthUnit::Inch),
            _ => None,
        }
    }

    pub fn to_meters(self) -> f64 {
        match self {
            LengthUnit::Meter => 1.0,
            LengthUnit::Millimeter => 0.001,
            LengthUnit::Centimeter => 0.01,
            LengthUnit::Foot => 0.3048,
            LengthUnit::Inch => 0.0254,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            LengthUnit::Meter => "m",
            LengthUnit::Millimeter => "mm",
            LengthUnit::Centimeter => "cm",
            LengthUnit::Foot => "ft",
            LengthUnit::Inch => "in",
        }
    }
}


#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AngleUnit {
    Radian,
    Degree,
}

impl AngleUnit {
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "rad" | "radian" | "radians" => Some(AngleUnit::Radian),
            "deg" | "degree" | "degrees" => Some(AngleUnit::Degree),
            _ => None,
        }
    }

    pub fn to_radians(self) -> f64 {
        match self {
            AngleUnit::Radian => 1.0,
            AngleUnit::Degree => std::f64::consts::PI / 180.0,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            AngleUnit::Radian => "rad",
            AngleUnit::Degree => "deg",
        }
    }
}

/// Units declared on an incoming packet.  Stored verbatim as part of the
/// import record alongside the normalized versions used for derived data.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct UnitSpec {
    pub length: LengthUnit,
    pub angle: AngleUnit,
}

impl UnitSpec {
    pub fn si() -> Self {
        UnitSpec {
            length: LengthUnit::Meter,
            angle: AngleUnit::Radian,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_lengths_and_angles() {
        assert!((LengthUnit::Millimeter.to_meters() - 0.001).abs() < 1e-15);
        assert!((LengthUnit::Foot.to_meters() - 0.3048).abs() < 1e-15);
        assert!((AngleUnit::Degree.to_radians() * 180.0 - std::f64::consts::PI).abs() < 1e-12);
        assert!(LengthUnit::parse("furlong").is_none());
        assert_eq!(LengthUnit::parse("MM"), Some(LengthUnit::Millimeter));
    }
}
