//! Minimal 3D geometry: vectors, quaternions, rigid transforms and 6x6
//! covariance propagation.  Coordinate order is always [x, y, z].

use serde::{Deserialize, Serialize};

pub type Vec3 = [f64; 3];

pub fn vadd(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn vsub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn vscale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn vdot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn vcross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn vnorm(a: Vec3) -> f64 {
    vdot(a, a).sqrt()
}

/// Quaternion stored as [w, x, y, z].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Quat(pub [f64; 4]);

impl Quat {
    pub fn identity() -> Self {
        Quat([1.0, 0.0, 0.0, 0.0])
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let n = vnorm(axis);
        if n < 1e-15 || angle.abs() < 1e-15 {
            return Quat::identity();
        }
        let u = vscale(axis, 1.0 / n);
        let h = angle * 0.5;
        let s = h.sin();
        Quat([h.cos(), u[0] * s, u[1] * s, u[2] * s])
    }

    /// Rotation from Euler roll/pitch/yaw (XYZ extrinsic) in radians.
    pub fn from_euler_xyz(roll: f64, pitch: f64, yaw: f64) -> Self {
        let qx = Quat::from_axis_angle([1.0, 0.0, 0.0], roll);
        let qy = Quat::from_axis_angle([0.0, 1.0, 0.0], pitch);
        let qz = Quat::from_axis_angle([0.0, 0.0, 1.0], yaw);
        qz.mul(&qy).mul(&qx)
    }

    pub fn mul(&self, o: &Quat) -> Self {
        let (w0, x0, y0, z0) = (self.0[0], self.0[1], self.0[2], self.0[3]);
        let (w1, x1, y1, z1) = (o.0[0], o.0[1], o.0[2], o.0[3]);
        Quat([
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ])
    }

    pub fn conjugate(&self) -> Self {
        Quat([self.0[0], -self.0[1], -self.0[2], -self.0[3]])
    }

    pub fn normalize(&self) -> Self {
        let n = (self.0[0] * self.0[0]
            + self.0[1] * self.0[1]
            + self.0[2] * self.0[2]
            + self.0[3] * self.0[3])
            .sqrt();
        if n < 1e-15 {
            return Quat::identity();
        }
        Quat([
            self.0[0] / n,
            self.0[1] / n,
            self.0[2] / n,
            self.0[3] / n,
        ])
    }

    pub fn rotate(&self, v: Vec3) -> Vec3 {
        let q = Quat([0.0, v[0], v[1], v[2]]);
        let r = self.mul(&q).mul(&self.conjugate());
        [r.0[1], r.0[2], r.0[3]]
    }

    pub fn rotation_matrix(&self) -> [[f64; 3]; 3] {
        let q = self.normalize().0;
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }

    pub fn inverse(&self) -> Self {
        let n2 = self.0[0] * self.0[0]
            + self.0[1] * self.0[1]
            + self.0[2] * self.0[2]
            + self.0[3] * self.0[3];
        let c = self.conjugate();
        Quat([c.0[0] / n2, c.0[1] / n2, c.0[2] / n2, c.0[3] / n2])
    }
}

/// Rigid transform: p_target = R * p_source + t.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Iso {
    pub q: Quat,
    pub t: Vec3,
}

impl Iso {
    pub fn identity() -> Self {
        Iso {
            q: Quat::identity(),
            t: [0.0, 0.0, 0.0],
        }
    }

    pub fn new(q: Quat, t: Vec3) -> Self {
        Iso {
            q: q.normalize(),
            t,
        }
    }

    pub fn apply(&self, p: Vec3) -> Vec3 {
        vadd(self.q.rotate(p), self.t)
    }

    /// self * o: apply o first, then self.
    pub fn compose(&self, o: &Iso) -> Self {
        Iso {
            q: self.q.mul(&o.q),
            t: vadd(self.q.rotate(o.t), self.t),
        }
    }

    pub fn inverse(&self) -> Self {
        let qi = self.q.inverse();
        Iso {
            t: vscale(qi.rotate(self.t), -1.0),
            q: qi,
        }
    }
}

/// 3x3 skew symmetric matrix for a rotation vector.
pub fn skew(v: Vec3) -> [[f64; 3]; 3] {
    [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]
}

pub fn mat3_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

/// A symmetric 6x6 covariance matrix, block order [translation(3), rotation(3)].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Cov6(pub [[f64; 6]; 6]);

impl Cov6 {
    pub fn zero() -> Self {
        Cov6([[0.0; 6]; 6])
    }

    pub fn diagonal(t: Vec3, r: Vec3) -> Self {
        let mut m = [[0.0; 6]; 6];
        for i in 0..3 {
            m[i][i] = t[i];
            m[i + 3][i + 3] = r[i];
        }
        Cov6(m)
    }

    pub fn is_positive_definite(&self) -> bool {
        cholesky6(&self.0).is_some()
    }

    /// Scalar "precision" cost: trace of the full 6x6 covariance.
    /// Smaller is better.  Used for deterministic route selection.
    pub fn trace_cost(&self) -> f64 {
        let mut s = 0.0;
        for i in 0..6 {
            s += self.0[i][i].max(0.0);
        }
        s
    }
}

/// Cholesky lower-triangle factor L with A = L L^T.  Returns None when the
/// matrix is not numerically positive definite (i.e. singular covariance).
pub fn cholesky6(a: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    let mut l = [[0.0f64; 6]; 6];
    let scale = (0..6)
        .map(|i| a[i][i].abs())
        .fold(1e-30, f64::max);
    // Relative tolerance: a covariance is rejected when its smallest pivot is
    // below ~1e-12 of its largest pivot, or when a pivot goes negative.
    let tol = 1e-12 * scale;
    for i in 0..6 {
        for j in 0..=i {
            let mut sum = a[i][j];
            for k in 0..j {
                sum -= l[i][k] * l[j][k];
            }
            if i == j {
                if !(sum > tol) {
                    return None;
                }
                l[i][i] = sum.sqrt();
            } else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    Some(l)
}

/// Generic dense matrix helpers used for point-covariance propagation.
pub fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    let m = b[0].len();
    let k = b.len();
    let mut r = vec![vec![0.0; m]; n];
    for i in 0..n {
        for j in 0..m {
            let mut s = 0.0;
            for p in 0..k {
                s += a[i][p] * b[p][j];
            }
            r[i][j] = s;
        }
    }
    r
}

pub fn mat_t(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    let m = a[0].len();
    let mut r = vec![vec![0.0; n]; m];
    for i in 0..n {
        for j in 0..m {
            r[j][i] = a[i][j];
        }
    }
    r
}

/// Jacobian of q.rotate(p) w.r.t. the *rotation vector* (small-angle) params,
/// evaluated at the current orientation: R * (-[p]_x).
pub fn rot_jac_wrt_angle(r: &[[f64; 3]; 3], p: Vec3) -> [[f64; 3]; 3] {
    let s = skew(p);
    let mut j = [[0.0; 3]; 3];
    for i in 0..3 {
        for k in 0..3 {
            let mut v = 0.0;
            for n in 0..3 {
                v += r[i][n] * (-s[n][k]);
            }
            j[i][k] = v;
        }
    }
    j
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quat_roundtrip_and_compose() {
        let q = Quat::from_axis_angle([0.0, 0.0, 1.0], 0.5);
        let v = [1.0, 0.0, 0.0];
        let w = q.rotate(v);
        assert!((w[0] - 0.5f64.cos()).abs() < 1e-12);
        assert!((w[1] - 0.5f64.sin()).abs() < 1e-12);

        let a = Iso::new(Quat::from_axis_angle([0.0, 0.0, 1.0], 0.2), [1.0, 0.0, 0.0]);
        let ai = a.inverse();
        let id = a.compose(&ai);
        assert!((id.q.0[0] - 1.0).abs() < 1e-12);
        assert!(vnorm(id.t) < 1e-12);
    }

    #[test]
    fn cholesky_detects_singular() {
        let good = Cov6::diagonal([1e-6; 3], [1e-8; 3]);
        assert!(good.is_positive_definite());
        let mut bad = [[0.0; 6]; 6];
        for i in 0..5 {
            bad[i][i] = 1.0;
        }
        assert!(cholesky6(&bad).is_none());
    }
}
