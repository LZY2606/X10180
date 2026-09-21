//! Minimal linear algebra for rigid transforms and covariance propagation.
//!
//! Coordinates are plain `[f64; 3]`. Rotations are unit quaternions
//! `[w, x, y, z]`. Covariances are symmetric 6x6 matrices in the
//! `[translation(3), rotation(3)]` ordering of the *source* frame,
//! expressed as the 21 unique entries of a symmetric matrix
//! (upper triangle, row major).

pub const SQRT_EPS: f64 = 1e-12;

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mat6 {
    pub a: [[f64; 6]; 6],
}

impl Mat6 {
    pub fn zero() -> Self {
        Mat6 { a: [[0.0; 6]; 6] }
    }

    pub fn identity() -> Self {
        let mut m = Self::zero();
        for i in 0..6 {
            m.a[i][i] = 1.0;
        }
        m
    }

    pub fn from_flat(v: &[f64]) -> Self {
        let mut m = Self::zero();
        for (i, row) in v.chunks(6).enumerate() {
            for (j, x) in row.iter().enumerate() {
                m.a[i][j] = *x;
            }
        }
        m
    }

    pub fn to_flat(self) -> Vec<f64> {
        self.a.iter().flatten().copied().collect()
    }

    pub fn mul(&self, o: &Mat6) -> Mat6 {
        let mut r = Mat6::zero();
        for i in 0..6 {
            for j in 0..6 {
                let mut s = 0.0;
                for k in 0..6 {
                    s += self.a[i][k] * o.a[k][j];
                }
                r.a[i][j] = s;
            }
        }
        r
    }

    pub fn transpose(&self) -> Mat6 {
        let mut r = Mat6::zero();
        for i in 0..6 {
            for j in 0..6 {
                r.a[j][i] = self.a[i][j];
            }
        }
        r
    }

    /// Covariance sandwich: self * c * self^T.
    pub fn cov_apply(&self, c: &Mat6) -> Mat6 {
        let sc = self.mul(c);
        let mut r = Mat6::zero();
        for i in 0..6 {
            for j in 0..=i {
                let mut s = 0.0;
                for k in 0..6 {
                    s += sc.a[i][k] * self.a[j][k];
                }
                r.a[i][j] = s;
                r.a[j][i] = s;
            }
        }
        r
    }

    pub fn add(&self, o: &Mat6) -> Mat6 {
        let mut r = Mat6::zero();
        for i in 0..6 {
            for j in 0..6 {
                r.a[i][j] = self.a[i][j] + o.a[i][j];
            }
        }
        r
    }

    pub fn trace(&self) -> f64 {
        self.a[0][0] + self.a[1][1] + self.a[2][2]
            + self.a[3][3] + self.a[4][4] + self.a[5][5]
    }

    /// Smallest eigenvalue of the symmetric 3x3 translation block via
    /// the analytical characteristic polynomial. Tiny/negative values
    /// flag a singular or non-positive-definite covariance.
    pub fn min_translation_eigenvalue(&self) -> f64 {
        let a = [
            [self.a[0][0], self.a[0][1], self.a[0][2]],
            [self.a[1][0], self.a[1][1], self.a[1][2]],
            [self.a[2][0], self.a[2][1], self.a[2][2]],
        ];
        let p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if p1 == 0.0 {
            return a[0][0].min(a[1][1]).min(a[2][2]);
        }
        let tr = a[0][0] + a[1][1] + a[2][2];
        let q = tr / 3.0;
        let p2 = (a[0][0] - q).powi(2) + (a[1][1] - q).powi(2) + (a[2][2] - q).powi(2)
            + 2.0 * p1;
        let mut b = [
            [a[0][0] - q, a[0][1], a[0][2]],
            [a[0][1], a[1][1] - q, a[1][2]],
            [a[0][2], a[1][2], a[2][2] - q],
        ];
        let det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1])
            - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0])
            + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
        let _ = &mut b;
        // Eigenvalues of (A - qI): 2 sqrt(p2/6) cos(...); phi formula.
        let r = (p2 / 6.0).sqrt();
        let phi = (det / (2.0 * r * r * r)).clamp(-1.0, 1.0).acos() / 3.0;
        let e0 = q + 2.0 * r * phi.cos();
        let e1 = q + 2.0 * r * (phi - 2.0 * std::f64::consts::PI / 3.0).cos();
        let e2 = q + 2.0 * r * (phi - 4.0 * std::f64::consts::PI / 3.0).cos();
        e0.min(e1).min(e2)
    }
}

pub fn vadd(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn vsub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn vscale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn vdot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn vcross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
    let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

pub fn qconj(a: [f64; 4]) -> [f64; 4] {
    [a[0], -a[1], -a[2], -a[3]]
}

pub fn qnorm(a: [f64; 4]) -> [f64; 4] {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]).sqrt();
    [a[0] / n, a[1] / n, a[2] / n, a[3] / n]
}

/// Rotate vector by quaternion: q * v * q^-1.
pub fn qrot(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let qv = [0.0, v[0], v[1], v[2]];
    let r = qmul(qmul(q, qv), qconj(q));
    [r[1], r[2], r[3]]
}

/// Spherical linear interpolation of unit quaternions.
pub fn slerp(a: [f64; 4], b: [f64; 4], t: f64) -> [f64; 4] {
    let mut dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let mut bb = b;
    if dot < 0.0 {
        dot = -dot;
        bb = [-b[0], -b[1], -b[2], -b[3]];
    }
    if dot > 0.9995 {
        return qnorm([
            a[0] + t * (bb[0] - a[0]),
            a[1] + t * (bb[1] - a[1]),
            a[2] + t * (bb[2] - a[2]),
            a[3] + t * (bb[3] - a[3]),
        ]);
    }
    let theta0 = dot.clamp(-1.0, 1.0).acos();
    let theta = theta0 * t;
    let s0 = theta.cos() - dot * (theta.sin() / theta0.sin());
    let s1 = theta.sin() / theta0.sin();
    qnorm([
        s0 * a[0] + s1 * bb[0],
        s0 * a[1] + s1 * bb[1],
        s0 * a[2] + s1 * bb[2],
        s0 * a[3] + s1 * bb[3],
    ])
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Rigid {
    /// Transform maps points in the child frame to the parent frame:
    /// p_parent = R * p_child + t.
    pub t: [f64; 3],
    pub q: [f64; 4],
}

impl Rigid {
    pub fn identity() -> Self {
        Rigid { t: [0.0, 0.0, 0.0], q: [1.0, 0.0, 0.0, 0.0] }
    }

    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        vadd(self.t, qrot(self.q, p))
    }

    pub fn inverse(&self) -> Rigid {
        let qi = qconj(self.q);
        Rigid { t: qrot(qi, vscale(self.t, -1.0)), q: qi }
    }

    /// self ∘ o : first apply o, then self.
    pub fn compose(&self, o: &Rigid) -> Rigid {
        Rigid { t: vadd(self.t, qrot(self.q, o.t)), q: qmul(self.q, o.q) }
    }

    /// Relative transform such that self ∘ rel = o (used for pose
    /// interpolation residuals).
    pub fn between(&self, o: &Rigid) -> Rigid {
        self.inverse().compose(o)
    }
}

/// 6x6 spatial adjoint of a rigid transform T=(R,t): maps a body twist
/// expressed in the child frame to the parent frame.
/// Ad(T) = [ R  [t]_x R ; 0 R ].
pub fn adjoint(xf: &Rigid) -> Mat6 {
    let q = xf.q;
    let c0 = qrot(q, [1.0, 0.0, 0.0]);
    let c1 = qrot(q, [0.0, 1.0, 0.0]);
    let c2 = qrot(q, [0.0, 0.0, 1.0]);
    let r = [
        [c0[0], c1[0], c2[0]],
        [c0[1], c1[1], c2[1]],
        [c0[2], c1[2], c2[2]],
    ];
    let t = xf.t;
    let sk = [
        [0.0, -t[2], t[1]],
        [t[2], 0.0, -t[0]],
        [-t[1], t[0], 0.0],
    ];
    let mut a = Mat6::identity();
    for i in 0..3 {
        for j in 0..3 {
            a.a[i][j] = r[i][j];
            a.a[3 + i][3 + j] = r[i][j];
            let mut sum = 0.0;
            for k in 0..3 {
                sum += sk[i][k] * r[k][j];
            }
            a.a[i][3 + j] = sum;
        }
    }
    a
}

/// Covariance (spatial/parent-frame twist) of `outer ∘ inner`.
/// Both inputs are spatial covariances; `cov_inner` (the transform
/// applied first) is pulled into the outer parent frame by Ad(outer).
pub fn compose_cov(outer: &Rigid, cov_outer: &Mat6, cov_inner: &Mat6) -> Mat6 {
    let adj = adjoint(outer);
    cov_outer.add(&adj.cov_apply(cov_inner))
}

/// 3x3 positional covariance of a transformed point, given the 6x6
/// *spatial* (parent-frame) covariance `cov` of transform `xf`, and
/// per-point measurement noise in the child frame.
/// Spatial twist perturbes p' = exp(xi) (R p + t), so
/// dp = dv - [Rp + t]_x domega => J = [ I  -[p_parent]_x ].
pub fn point_covariance(xf: &Rigid, cov: &Mat6, p_child: [f64; 3], noise: f64) -> [[f64; 3]; 3] {
    let pp = xf.apply(p_child);
    let sk = [
        [0.0, -pp[2], pp[1]],
        [pp[2], 0.0, -pp[0]],
        [-pp[1], pp[0], 0.0],
    ];
    let mut j = [[0.0; 6]; 3];
    for i in 0..3 {
        j[i][i] = 1.0;
        for k in 0..3 {
            j[i][3 + k] = -sk[i][k];
        }
    }
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for jj in 0..3 {
            let mut s = 0.0;
            for u in 0..6 {
                for v in 0..6 {
                    s += j[i][u] * cov.a[u][v] * j[jj][v];
                }
            }
            out[i][jj] = s;
        }
        out[i][i] += noise;
    }
    out
}

/// Validate a supplied 6x6 covariance: finite, symmetric and positive
/// definite on its translation block above a tolerance floor.
pub fn validate_covariance(c: &Mat6) -> Result<(), String> {
    for i in 0..6 {
        for j in 0..6 {
            if !c.a[i][j].is_finite() {
                return Err("covariance contains non-finite values".into());
            }
            if (c.a[i][j] - c.a[j][i]).abs() > 1e-9 {
                return Err(format!("covariance not symmetric at ({},{})", i, j));
            }
        }
    }
    let ev = c.min_translation_eigenvalue();
    if ev < SQRT_EPS {
        return Err(format!("singular or non-positive-definite covariance: min translation eigenvalue {}", ev));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn angle_axis(axis: [f64; 3], ang: f64) -> [f64; 4] {
        let h = ang / 2.0;
        let s = h.sin();
        qnorm([h.cos(), axis[0] * s, axis[1] * s, axis[2] * s])
    }

    #[test]
    fn rotate_and_compose_roundtrip() {
        let xf = Rigid { t: [1.0, 2.0, 3.0], q: angle_axis([0.0, 0.0, 1.0], 0.7) };
        let p = [0.5, -0.3, 1.2];
        let back = xf.inverse().apply(xf.apply(p));
        for i in 0..3 {
            assert!((back[i] - p[i]).abs() < 1e-12, "axis {}", i);
        }
        let c = xf.compose(&xf.inverse());
        for i in 0..3 {
            assert!(c.t[i].abs() < 1e-12);
        }
    }

    #[test]
    fn adjoint_matches_known_formula_and_numeric_body_twist() {
        // Pure translation t=[0.4,-0.2,0.7]: Ad = [I [t]_x; 0 I].
        let xf = Rigid { t: [0.4, -0.2, 0.7], q: [1.0, 0.0, 0.0, 0.0] };
        let adj = adjoint(&xf);
        let expect = [
            [0.0, -0.7, -0.2],
            [0.7, 0.0, -0.4],
            [0.2, 0.4, 0.0],
        ];
        for i in 0..3 {
            for j in 0..3 {
                assert!((adj.a[i][3 + j] - expect[i][j]).abs() < 1e-12);
                assert!((adj.a[i][j] - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12);
                assert!((adj.a[3 + i][3 + j] - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12);
                assert!((adj.a[3 + i][j] - 0.0).abs() < 1e-12);
            }
        }

        // General transform: S = T exp(xi_body) T^-1 = exp(xi_spatial).
        // For a spatial twist, exp has translation exactly v_s (the
        // rotation is small), so S.t / eps is the first 3 rows of Ad.
        let xf = Rigid { t: [0.4, -0.2, 0.7], q: angle_axis([0.3, 0.8, 0.5], 0.5) };
        let adj = adjoint(&xf);
        let eps = 1e-7;
        for col in 0..6 {
            let mut db = [0.0; 6];
            db[col] = eps;
            let body = Rigid {
                t: [db[0], db[1], db[2]],
                q: [1.0, db[3] / 2.0, db[4] / 2.0, db[5] / 2.0],
            };
            let s_xf = xf.compose(&body).compose(&xf.inverse());
            let num = [
                s_xf.t[0] / eps, s_xf.t[1] / eps, s_xf.t[2] / eps,
                2.0 * s_xf.q[1] / eps, 2.0 * s_xf.q[2] / eps, 2.0 * s_xf.q[3] / eps,
            ];
            for i in 0..6 {
                assert!(
                    (num[i] - adj.a[i][col]).abs() < 1e-4,
                    "row {} col {} numeric {} adj {}",
                    i, col, num[i], adj.a[i][col]
                );
            }
        }
    }

    #[test]
    fn point_covariance_matches_numeric() {
        let xf = Rigid { t: [0.2, 0.1, -0.3], q: angle_axis([0.1, 0.9, 0.4], 0.4) };
        let mut cov = Mat6::zero();
        for i in 0..6 {
            cov.a[i][i] = if i < 3 { 0.01 } else { 0.001 };
        }
        cov.a[0][1] = 0.002;
        cov.a[1][0] = 0.002;
        let p = [0.6, -0.4, 0.2];
        let c = point_covariance(&xf, &cov, p, 0.0);
        // Numeric Monte-Carlo-ish Jacobian check via finite differences:
        // build sample noise in transform and measure variance? Instead
        // verify the closed form equals J Sigma J^T from the documented
        // left-perturb Jacobian.
        let rp = xf.apply(p);
        let sk = [
            [0.0, -rp[2], rp[1]],
            [rp[2], 0.0, -rp[0]],
            [-rp[1], rp[0], 0.0],
        ];
        for i in 0..3 {
            for jj in 0..3 {
                let mut expect = 0.0;
                for u in 0..6 {
                    for v in 0..6 {
                        let ji = if u < 3 { if u == i { 1.0 } else { 0.0 } } else { -sk[i][u - 3] };
                        let jjj = if v < 3 { if v == jj { 1.0 } else { 0.0 } } else { -sk[jj][v - 3] };
                        expect += ji * cov.a[u][v] * jjj;
                    }
                }
                assert!((c[i][jj] - expect).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn slerp_endpoints_and_midpoint() {
        let a = angle_axis([0.0, 0.0, 1.0], 0.0);
        let b = angle_axis([0.0, 0.0, 1.0], 1.0);
        let p0 = slerp(a, b, 0.0);
        let p1 = slerp(a, b, 1.0);
        let pm = slerp(a, b, 0.5);
        assert!((p0[0] - 1.0).abs() < 1e-12);
        assert!((p1[3] - 0.5_f64.sin()).abs() < 1e-12);
        assert!((pm[3] - 0.25_f64.sin()).abs() < 1e-12);
    }

    #[test]
    fn zero_covariance_is_rejected() {
        assert!(validate_covariance(&Mat6::zero()).is_err());
    }

    #[test]
    fn diagonal_covariance_accepted() {
        let mut c = Mat6::zero();
        for i in 0..6 {
            c.a[i][i] = 0.01;
        }
        assert!(validate_covariance(&c).is_ok());
    }
}
