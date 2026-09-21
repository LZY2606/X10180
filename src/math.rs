//! 刚体变换与误差传播的基础数学。所有几何比较使用显式容差。

pub const GEOM_EPS: f64 = 1e-9;
pub const PSD_TOL: f64 = 1e-12;

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    pub fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
    pub fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn approx_eq(self, o: Self, eps: f64) -> bool {
        (self.x - o.x).abs() <= eps && (self.y - o.y).abs() <= eps && (self.z - o.z).abs() <= eps
    }
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub fn identity() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let n = axis.norm();
        if n < GEOM_EPS {
            return Self::identity();
        }
        let h = 0.5 * angle;
        let s = h.sin() / n;
        Self { w: h.cos(), x: axis.x * s, y: axis.y * s, z: axis.z * s }.normalized()
    }
    pub fn normalized(self) -> Self {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n < GEOM_EPS {
            return Self::identity();
        }
        Self { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
    }
    pub fn conj(self) -> Self {
        Self { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }
    pub fn mul(self, o: Self) -> Self {
        Self {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let uv = qv.cross(v);
        let uuv = qv.cross(uv);
        v.add(uv.scale(2.0 * self.w)).add(uuv.scale(2.0))
    }
    /// 旋转矩阵（行主序）。
    pub fn to_mat3(self) -> [[f64; 3]; 3] {
        let q = self.normalized();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    }
    /// 与另一旋转之间的夹角（弧度，[0, pi]）。
    pub fn angle_to(self, o: Self) -> f64 {
        let d = (self.w * o.w + self.x * o.x + self.y * o.y + self.z * o.z).abs().min(1.0);
        2.0 * d.acos()
    }
    /// 球面线性插值。
    pub fn slerp(self, o: Self, t: f64) -> Self {
        let mut dot = self.w * o.w + self.x * o.x + self.y * o.y + self.z * o.z;
        let mut other = o;
        if dot < 0.0 {
            dot = -dot;
            other = Self { w: -o.w, x: -o.x, y: -o.y, z: -o.z };
        }
        if dot > 0.9995 {
            return Self {
                w: self.w + t * (other.w - self.w),
                x: self.x + t * (other.x - self.x),
                y: self.y + t * (other.y - self.y),
                z: self.z + t * (other.z - self.z),
            }
            .normalized();
        }
        let theta = dot.clamp(-1.0, 1.0).acos();
        let s = theta.sin();
        let a = ((1.0 - t) * theta).sin() / s;
        let b = (t * theta).sin() / s;
        Self {
            w: a * self.w + b * other.w,
            x: a * self.x + b * other.x,
            y: a * self.y + b * other.y,
            z: a * self.z + b * other.z,
        }
    }
}

/// 刚体变换 SE(3)：先旋转后平移，apply(p) = R*p + t。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SE3 {
    pub rot: Quat,
    pub trans: Vec3,
}

impl SE3 {
    pub fn identity() -> Self {
        Self { rot: Quat::identity(), trans: Vec3::default() }
    }
    pub fn new(rot: Quat, trans: Vec3) -> Self {
        Self { rot: rot.normalized(), trans }
    }
    /// self ∘ other：先应用 other，再应用 self。
    pub fn compose(self, other: Self) -> Self {
        Self {
            rot: (self.rot.mul(other.rot)).normalized(),
            trans: self.rot.rotate(other.trans).add(self.trans),
        }
    }
    pub fn inverse(self) -> Self {
        let r = self.rot.conj();
        Self { rot: r, trans: r.rotate(self.trans.scale(-1.0)) }
    }
    pub fn apply(self, p: Vec3) -> Vec3 {
        self.rot.rotate(p).add(self.trans)
    }
    /// 与恒等变换的残差：平移范数 + 旋转角（弧度）。
    pub fn residual_to_identity(self) -> f64 {
        self.trans.norm() + self.rot.angle_to(Quat::identity())
    }
    pub fn approx_eq(self, o: Self, eps: f64) -> bool {
        self.trans.approx_eq(o.trans, eps) && self.rot.angle_to(o.rot) <= eps
    }
}

/// 6x6 矩阵，李代数顺序 [旋转(3), 平移(3)]。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mat6 {
    pub m: [[f64; 6]; 6],
}

impl Mat6 {
    pub fn zero() -> Self {
        Self { m: [[0.0; 6]; 6] }
    }
    pub fn from_diag(d: [f64; 6]) -> Self {
        let mut r = Self::zero();
        for i in 0..6 {
            r.m[i][i] = d[i];
        }
        r
    }
    pub fn add(self, o: Self) -> Self {
        let mut r = Self::zero();
        for i in 0..6 {
            for j in 0..6 {
                r.m[i][j] = self.m[i][j] + o.m[i][j];
            }
        }
        r
    }
    pub fn mul(self, o: Self) -> Self {
        let mut r = Self::zero();
        for i in 0..6 {
            for j in 0..6 {
                let mut s = 0.0;
                for k in 0..6 {
                    s += self.m[i][k] * o.m[k][j];
                }
                r.m[i][j] = s;
            }
        }
        r
    }
    pub fn transpose(self) -> Self {
        let mut r = Self::zero();
        for i in 0..6 {
            for j in 0..6 {
                r.m[i][j] = self.m[j][i];
            }
        }
        r
    }
    pub fn trace(self) -> f64 {
        (0..6).map(|i| self.m[i][i]).sum()
    }
    /// SE(3) 伴随矩阵：Ad(T) = [[R, 0], [[t]x R, R]]（顺序 [rot, trans]）。
    pub fn adjoint(t: &SE3) -> Self {
        let r = t.rot.to_mat3();
        let tx = [
            [0.0, -t.trans.z, t.trans.y],
            [t.trans.z, 0.0, -t.trans.x],
            [-t.trans.y, t.trans.x, 0.0],
        ];
        let mut out = Self::zero();
        for i in 0..3 {
            for j in 0..3 {
                out.m[i][j] = r[i][j];
                let mut s = 0.0;
                for k in 0..3 {
                    s += tx[i][k] * r[k][j];
                }
                out.m[i + 3][j] = s;
                out.m[i + 3][j + 3] = r[i][j];
            }
        }
        out
    }
    /// 组合变换的一阶误差传播：T = a ∘ b 时 Σ = Σa + Ad(a) Σb Ad(a)ᵀ。
    pub fn propagate(a: &SE3, sa: &Self, sb: &Self) -> Self {
        let ad = Self::adjoint(a);
        sa.add(ad.mul(*sb).mul(ad.transpose()))
    }
    /// 对称半正定检查（允许奇异，即零特征值；负特征值超出容差则拒绝）。
    pub fn is_psd(&self, tol: f64) -> bool {
        for i in 0..6 {
            for j in 0..6 {
                if (self.m[i][j] - self.m[j][i]).abs() > 1e-6 {
                    return false;
                }
            }
        }
        // LDLᵀ（Cholesky 的半正定变体）
        let mut a = self.m;
        for k in 0..6 {
            if a[k][k] < -tol {
                return false;
            }
            if a[k][k] <= tol {
                // 零主元：要求整列（剩余部分）也为零，否则不定。
                for i in (k + 1)..6 {
                    if a[i][k].abs() > 1e-9 {
                        return false;
                    }
                }
                continue;
            }
            let d = a[k][k];
            for i in (k + 1)..6 {
                let f = a[i][k] / d;
                for j in (k + 1)..6 {
                    a[i][j] -= f * a[k][j];
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn se3_roundtrip_with_tolerance() {
        let t = SE3::new(
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.7),
            Vec3::new(1.0, -2.0, 3.0),
        );
        let p = Vec3::new(0.5, 1.5, -0.25);
        let back = t.inverse().apply(t.apply(p));
        assert!(back.approx_eq(p, 1e-9), "roundtrip drift: {:?}", back.sub(p));
    }

    #[test]
    fn compose_matches_apply() {
        let a = SE3::new(Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), 0.3), Vec3::new(1.0, 0.0, 0.0));
        let b = SE3::new(Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), -0.2), Vec3::new(0.0, 2.0, 1.0));
        let p = Vec3::new(3.0, -1.0, 0.5);
        assert!(a.compose(b).apply(p).approx_eq(a.apply(b.apply(p)), 1e-9));
    }

    #[test]
    fn psd_accepts_singular_rejects_indefinite() {
        assert!(Mat6::from_diag([1.0, 0.0, 2.0, 0.0, 3.0, 1.0]).is_psd(PSD_TOL));
        assert!(!Mat6::from_diag([1.0, -1e-6, 2.0, 0.0, 3.0, 1.0]).is_psd(PSD_TOL));
    }
}
