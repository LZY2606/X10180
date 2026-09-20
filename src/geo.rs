//! SE(3) 刚体变换、6 自由度误差传播与单位换算。
//!
//! 内部顺序统一为“旋转 ω（对数/指数坐标，与平移同序）”。
//! 6x6 协方差采用块布局 [ω ω, ω t; t ω, t t]。

use nalgebra::{
    Matrix3, Matrix6, Point3, UnitQuaternion, Vector3, Vector6,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Se3 {
    /// 四元数 [x, y, z, w]
    pub q: [f64; 4],
    /// 平移（米）
    pub t: [f64; 3],
}

impl Se3 {
    pub fn identity() -> Self {
        Self {
            q: [0.0, 0.0, 0.0, 1.0],
            t: [0.0; 3],
        }
    }

    pub fn from_iso(rot_xyz: [f64; 3], t: [f64; 3]) -> Self {
        let q = UnitQuaternion::from_scaled_axis(Vector3::new(rot_xyz[0], rot_xyz[1], rot_xyz[2]));
        Self {
            q: [q.i, q.j, q.k, q.w],
            t,
        }
    }

    pub fn uq(&self) -> UnitQuaternion<f64> {
        let q = nalgebra::Quaternion::new(self.q[3], self.q[0], self.q[1], self.q[2]);
        UnitQuaternion::from_quaternion(q)
    }

    pub fn v(&self) -> Vector3<f64> {
        Vector3::new(self.t[0], self.t[1], self.t[2])
    }

    pub fn compose(&self, other: &Se3) -> Se3 {
        let q = self.uq() * other.uq();
        let t = self.uq() * other.v() + self.v();
        Se3 {
            q: [q.i, q.j, q.k, q.w],
            t: [t.x, t.y, t.z],
        }
    }

    pub fn inverse(&self) -> Se3 {
        let q = self.uq().inverse();
        let t = -(q * self.v());
        Se3 {
            q: [q.i, q.j, q.k, q.w],
            t: [t.x, t.y, t.z],
        }
    }

    pub fn transform_point(&self, p: &[f64; 3]) -> [f64; 3] {
        let r = self.uq() * Point3::new(p[0], p[1], p[2]).coords + self.v();
        [r.x, r.y, r.z]
    }

    pub fn is_finite(&self) -> bool {
        self.q.iter().chain(self.t.iter()).all(|v| v.is_finite())
    }
}

/// 残差（环路闭合误差）：平移范数与旋转角。
pub fn displacement(a: &Se3, b: &Se3) -> (f64, f64) {
    let d = a.compose(&b.inverse());
    let dt = d.v().norm();
    let ang = d.uq().scaled_axis().norm();
    (dt, ang)
}

/// SE(3) 的 6x6 伴随矩阵，顺序 [ω; v]。
pub fn adjoint(g: &Se3) -> Matrix6<f64> {
    let r = g.uq().to_rotation_matrix();
    let r = r.matrix();
    let th = skew(&g.v()) * r;
    let mut m = Matrix6::zeros();
    let zero = Matrix3::zeros();
    let blocks: [((usize, usize), &Matrix3<f64>); 4] = [
        ((0, 0), &r),
        ((0, 3), &zero),
        ((3, 0), &th),
        ((3, 3), &r),
    ];
    for ((i, j), b) in blocks {
        for r in 0..3 {
            for c in 0..3 {
                m[(i + r, j + c)] = b[(r, c)];
            }
        }
    }
    m
}

fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

/// 对 T 做 s∈[0,1] 插值（旋转 SLERP，平移线性）。
pub fn interpolate(a: &Se3, b: &Se3, s: f64) -> Se3 {
    let rel = a.uq().inverse() * b.uq();
    let axis = rel.scaled_axis();
    let q = a.uq() * UnitQuaternion::from_scaled_axis(axis * s);
    let t = a.v() + (b.v() - a.v()) * s;
    Se3 {
        q: [q.i, q.j, q.k, q.w],
        t: [t.x, t.y, t.z],
    }
}

/// 插值协方差：凸组合 + 两端不确定性的额外贡献，保证非奇异且为正定。
pub fn interpolate_cov(
    ca: &Matrix6<f64>,
    cb: &Matrix6<f64>,
    s: f64,
) -> Matrix6<f64> {
    let mix = ca * (1.0 - s) + cb * s;
    let spread = cb - ca;
    &mix + &(&spread * s * (1.0 - s) * 0.25) + Matrix6::identity() * 1e-12
}

/// 复合 g = a∘b 时的误差传播：Σ_g = Ad_bᵀ Σ_a Ad_b + Σ_b。
pub fn compose_cov(
    g: &Se3,
    ca: &Matrix6<f64>,
    cb: &Matrix6<f64>,
) -> Matrix6<f64> {
    let ad = adjoint(&g.inverse());
    let adt = ad.transpose();
    &adt * ca * ad + cb + Matrix6::identity() * 1e-15
}

/// 点 p（body 系）经 g 变换后的 3x3 位置协方差，由 g 的 6x6 协方差传播。
///
/// 解析雅可比 J = [ -R [p]× | R ]（扰动静约定 R'≈R(I+[ω]×)）。
pub fn point_cov(g: &Se3, cov: &Matrix6<f64>, p_body: &[f64; 3]) -> Matrix3<f64> {
    let r = g.uq().to_rotation_matrix();
    let r = r.matrix();
    let p = Vector3::new(p_body[0], p_body[1], p_body[2]);
    let jw = -r * skew(&p);
    type M36 = nalgebra::Matrix<f64, nalgebra::Const<3>, nalgebra::Const<6>,
        nalgebra::ArrayStorage<f64, 3, 6>>;
    let mut j = M36::zeros();
    for i in 0..3 {
        for k in 0..3 {
            j[(i, k)] = jw[(i, k)];
            j[(i, k + 3)] = r[(i, k)];
        }
    }
    j * cov * j.transpose()
}

/// 6 维向量指数映射（小扰动用，含二阶近似）。
pub fn exp_se3(x: &Vector6<f64>) -> Se3 {
    let w = x.fixed_view::<3, 1>(0, 0).into_owned();
    let v = x.fixed_view::<3, 1>(3, 0).into_owned();
    Se3::from_iso([w.x, w.y, w.z], [v.x, v.y, v.z])
}

/// 路径精度评分：协方差迹越小越精确。
pub fn precision_score(c: &Matrix6<f64>) -> f64 {
    let mut tr = 0.0;
    for i in 0..6 {
        tr += c[(i, i)];
    }
    tr.max(0.0)
}

pub fn diag6(v: f64) -> Matrix6<f64> {
    Matrix6::identity() * v
}

/// 校验用户提供的协方差：有限、对称、正定、非奇异（最小特征值阈值）。
pub fn validate_cov(c: &Matrix6<f64>, min_eig: f64) -> Result<(), String> {
    for v in c.iter() {
        if !v.is_finite() {
            return Err("协方差含非有限值".into());
        }
    }
    let sym = c * 0.5 + c.transpose() * 0.5;
    if (sym - c).norm() > 1e-7 * (c.norm() + 1.0) {
        return Err("协方差不对称".into());
    }
    match nalgebra::SymmetricEigen::new(sym).eigenvalues.iter().copied().fold(Some(f64::INFINITY), |a, b| match a {
        Some(a) => Some(a.min(b)),
        None => None,
    }) {
        Some(min) if min >= min_eig => Ok(()),
        Some(min) => Err(format!(
            "协方差奇异或非正定：最小特征值 {min:.3e} < {min_eig:.0e}"
        )),
        None => Err("协方差特征分解失败".into()),
    }
}

/// 长度单位换算到米。
pub fn length_to_meters(v: f64, unit: &str) -> Result<f64, String> {
    Ok(match unit {
        "m" | "meter" | "meters" => v,
        "mm" => v * 1e-3,
        "cm" => v * 1e-2,
        "ft" | "foot" | "feet" => v * 0.3048,
        other => return Err(format!("未知长度单位 `{other}`（支持 m/mm/cm/ft）")),
    })
}

/// 解析 6 元素对角或 36 元素行优先协方差。
pub fn parse_cov(v: &serde_json::Value) -> Result<Matrix6<f64>, String> {
    let arr = v
        .as_array()
        .ok_or_else(|| "协方差必须是数组".to_string())?;
    let nums: Vec<f64> = arr
        .iter()
        .map(|x| x.as_f64().ok_or_else(|| "协方差元素必须是数字".to_string()))
        .collect::<Result<_, _>>()?;
    let m = match nums.len() {
        6 => {
            let mut m = Matrix6::zeros();
            for i in 0..6 {
                m[(i, i)] = nums[i];
            }
            m
        }
        36 => {
            let mut m = Matrix6::zeros();
            for i in 0..6 {
                for j in 0..6 {
                    m[(i, j)] = nums[i * 6 + j];
                }
            }
            m
        }
        n => return Err(format!("协方差需 6 或 36 个元素，得到 {n}")),
    };
    Ok(m)
}

pub fn row_major(c: &Matrix6<f64>) -> Vec<f64> {
    let mut v = Vec::with_capacity(36);
    for i in 0..6 {
        for j in 0..6 {
            v.push(c[(i, j)]);
        }
    }
    v
}

/// 误差传播链中的一步（供点来源链展示）。
#[derive(Debug, Clone, Serialize)]
pub struct PropagationStep {
    pub edge_id: String,
    pub kind: String,
    pub version: i64,
    pub valid_from: f64,
    pub valid_to: f64,
    pub se3: Se3,
    pub cov_trace: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose_inverse_roundtrip() {
        let a = Se3::from_iso([0.1, -0.2, 0.3], [1.0, 2.0, 3.0]);
        let b = Se3::from_iso([0.0, 0.4, -0.1], [-2.0, 0.5, 1.0]);
        let g = a.compose(&b);
        let back = g.compose(&b.inverse()).compose(&a.inverse());
        let p0 = Se3::identity();
        let (dt, ang) = displacement(&back, &p0);
        assert!(dt < 1e-10 && ang < 1e-10);
    }

    #[test]
    fn covariance_grows_and_rejects_singular() {
        let ca = diag6(1e-3);
        let cb = diag6(2e-3);
        let g = Se3::from_iso([0.1, 0.0, 0.0], [1.0, 0.0, 0.0]);
        let cg = compose_cov(&g, &ca, &cb);
        assert!(precision_score(&cg) > precision_score(&ca));
        let bad = Matrix6::zeros();
        assert!(validate_cov(&bad, 1e-8).is_err());
        assert!(validate_cov(&diag6(-1.0), 1e-8).is_err());
        assert!(validate_cov(&diag6(1e-3), 1e-8).is_ok());
    }

    #[test]
    fn point_cov_propagates_and_units() {
        let g = Se3::from_iso([0.0, 0.0, 0.1], [1.0, 0.0, 0.0]);
        let cov = diag6(1e-4);
        let pc = point_cov(&g, &cov, &[2.0, 0.0, 0.0]);
        for i in 0..3 {
            assert!(pc[(i, i)].is_finite() && pc[(i, i)] > 0.0);
        }
        assert!((length_to_meters(1000.0, "mm").unwrap() - 1.0).abs() < 1e-12);
        assert!((length_to_meters(1.0, "ft").unwrap() - 0.3048).abs() < 1e-12);
        assert!(length_to_meters(1.0, "cubit").is_err());
    }

    #[test]
    fn interpolation_midpoint() {
        let a = Se3::from_iso([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let b = Se3::from_iso([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let m = interpolate(&a, &b, 0.5);
        assert!((m.t[0] - 5.0).abs() < 1e-10);
        let ca = diag6(1e-4);
        let cb = diag6(3e-4);
        let cm = interpolate_cov(&ca, &cb, 0.5);
        assert!(validate_cov(&cm, 1e-12).is_ok());
    }
}
