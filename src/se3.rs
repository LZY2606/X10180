//! SE(3) 刚体变换与左扰动协方差传播。

use nalgebra::{
    DMatrix, Matrix3, Matrix6, Quaternion, UnitQuaternion, Vector3, Vector6,
};

pub type Iso = nalgebra::Isometry3<f64>;

/// 平移 + 四元数 (x,y,z,w) 构造变换。
pub fn iso(t: [f64; 3], q: [f64; 4]) -> Result<Iso, String> {
    let q = Quaternion::new(q[3], q[0], q[1], q[2]);
    if q.norm() < 0.5 {
        return Err("四元数幅值过小，无法归一化".into());
    }
    let uq = UnitQuaternion::from_quaternion(q.normalize());
    Ok(Iso::from_parts(
        nalgebra::Translation3::new(t[0], t[1], t[2]),
        uq,
    ))
}

pub fn params(tf: &Iso) -> ([f64; 3], [f64; 4]) {
    let t = tf.translation.vector;
    let q = tf.rotation.quaternion();
    ([t.x, t.y, t.z], [q.i, q.j, q.k, q.w])
}

/// 3x3 反对称矩阵。
fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

/// 左扰动伴随 Ad(T)，6x6：[[R, [t]x R], [0, R]]。
pub fn adjoint(tf: &Iso) -> Matrix6<f64> {
    let r = tf.rotation.to_rotation_matrix();
    let tr = skew(&tf.translation.vector) * r;
    let mut a = Matrix6::zeros();
    for i in 0..3 {
        for j in 0..3 {
            a[(i, j)] = r[(i, j)];
            a[(i + 3, j + 3)] = r[(i, j)];
            a[(i, j + 3)] = tr[(i, j)];
        }
    }
    a
}

/// C = A * B 的误差传播：
/// Sigma_C = Sigma_A + Ad(A) Sigma_B Ad(A)^T。
pub fn compose_cov(a: &Iso, sa: &Matrix6<f64>, b: &Iso, sb: &Matrix6<f64>) -> Matrix6<f64> {
    let _ = b;
    let ad = adjoint(a);
    sa + ad * sb * ad.transpose()
}

/// T_inv = T^{-1} 的 6x6 雅可比（左扰动）：diag 块 -R^T；
/// 左上平移块 +R^T [t]x。
pub fn inverse_jacobian(tf: &Iso) -> Matrix6<f64> {
    let rt = tf.rotation.to_rotation_matrix().transpose();
    let tl = rt * skew(&tf.translation.vector);
    let mut j = Matrix6::zeros();
    for i in 0..3 {
        for k in 0..3 {
            j[(i + 3, k + 3)] = -rt[(i, k)];
            j[(i, k + 3)] = tl[(i, k)];
            j[(i, k)] = -rt[(i, k)];
        }
    }
    j
}

pub fn invert_cov(tf: &Iso, s: &Matrix6<f64>) -> Matrix6<f64> {
    let j = inverse_jacobian(tf);
    j * s * j.transpose()
}

/// 检查 6x6 协方差：长度 36、对称、半正定（最小特征值 >= -tol）。
/// 严格正定通过 cholesky（对角必须为正）。
pub fn validate_cov(v: &[f64]) -> Result<Matrix6<f64>, String> {
    if v.is_empty() {
        return Ok(Matrix6::zeros());
    }
    if v.len() != 36 {
        return Err(format!("协方差必须为 6x6=36 个值，得到 {}", v.len()));
    }
    let m = DMatrix::from_row_slice(6, 6, v);
    let diff = (&m - &m.transpose()).abs().max();
    if diff > 1e-9 {
        return Err(format!("协方差不对称（最大偏差 {diff:.2e}）"));
    }
    let m6 = Matrix6::from_row_slice(v);
    let eig = m6.symmetric_eigen().eigenvalues;
    let min = eig.min();
    if !min.is_finite() {
        return Err("协方差含非有限值".into());
    }
    if min < -1e-9 {
        return Err(format!("协方差非半正定（最小特征值 {min:.3e}）"));
    }
    if m6.cholesky().is_none() {
        return Err(format!(
            "奇异协方差：无法 Cholesky 分解（最小特征值 {min:.3e}）"
        ));
    }
    Ok(m6)
}

/// SE(3) 指数映射：xi = [v; w] -> T。
pub fn se3_exp(xi: &Vector6<f64>) -> Iso {
    let v = Vector3::new(xi[0], xi[1], xi[2]);
    let w = Vector3::new(xi[3], xi[4], xi[5]);
    let theta = w.norm();
    let wx = skew(&w);
    let (rot_m, vmat) = if theta < 1e-9 {
        let r = Matrix3::identity() + &wx;
        let v = Matrix3::identity() + 0.5 * &wx;
        (r, v)
    } else {
        let r = Matrix3::identity()
            + (theta.sin() / theta) * &wx
            + ((1.0 - theta.cos()) / (theta * theta)) * (&wx * &wx);
        let v = Matrix3::identity()
            + ((1.0 - theta.cos()) / (theta * theta)) * &wx
            + ((theta - theta.sin()) / (theta * theta * theta)) * (&wx * &wx);
        (r, v)
    };
    let t = vmat * v;
    Iso::from_parts(
        nalgebra::Translation3::from(t),
        UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(rot_m)),
    )
}

/// SE(3) 对数映射：T -> xi = [v; w]。
pub fn se3_log(tf: &Iso) -> Vector6<f64> {
    let rm = tf.rotation.to_rotation_matrix();
    let w = tf.rotation.scaled_axis();
    let theta = w.norm();
    let wx = skew(&w);
    let v_inv = if theta < 1e-9 {
        Matrix3::identity() - 0.5 * &wx
    } else {
        let coeff = 1.0 / (theta * theta)
            * (1.0 - theta / (2.0 * (theta / 2.0).tan()));
        Matrix3::identity() - 0.5 * &wx + coeff * (&wx * &wx)
    };
    let v = v_inv * tf.translation.vector;
    Vector6::new(v.x, v.y, v.z, w.x, w.y, w.z)
}

/// SE(3) 左乘扰动：exp(xi) * tf。
fn left_retract(tf: &Iso, xi: &Vector6<f64>) -> Iso {
    se3_exp(xi) * tf
}

/// SE(3) 测地线插值 T(a) -> T(b)，alpha in [0,1]（左李）。
pub fn interpolate(a: &Iso, b: &Iso, alpha: f64) -> Iso {
    let xi = se3_log(&a.inv_mul(b));
    se3_exp(&(xi * alpha)) * a
}

/// 插值协方差：对相对变换用数值雅可比（左扰动）做线性传播，
/// Sigma_rel = J Sa J^T + K Sb K^T。
pub fn interp_cov(a: &Iso, sa: &Matrix6<f64>, b: &Iso, sb: &Matrix6<f64>, alpha: f64) -> Matrix6<f64> {
    let eps = 1e-6;
    let base = interpolate(a, b, alpha);
    let base_rel = a.inv_mul(&base);
    let mut ja = Matrix6::zeros();
    let mut jb = Matrix6::zeros();
    for k in 0..6 {
        let mut da = Vector6::zeros();
        da[k] = eps;
        let ap = left_retract(a, &da);
        let rp = ap.inv_mul(&interpolate(&ap, b, alpha));
        let drp = log_delta(&base_rel, &rp);
        for i in 0..6 {
            ja[(i, k)] = drp[i] / eps;
        }
        let mut db = Vector6::zeros();
        db[k] = eps;
        let bp = left_retract(b, &db);
        let rp2 = a.inv_mul(&interpolate(a, &bp, alpha));
        let drp2 = log_delta(&base_rel, &rp2);
        for i in 0..6 {
            jb[(i, k)] = drp2[i] / eps;
        }
    }
    ja * sa * ja.transpose() + jb * sb * jb.transpose()
}

/// 左对数差：log(base^{-1} * other) 的 6 维表示（平移+轴角）。
fn log_delta(base: &Iso, other: &Iso) -> Vector6<f64> {
    let d = base.inv_mul(other);
    let w = d.rotation.scaled_axis();
    let mut out = Vector6::zeros();
    out[0] = d.translation.x;
    out[1] = d.translation.y;
    out[2] = d.translation.z;
    out[3] = w.x;
    out[4] = w.y;
    out[5] = w.z;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_inverse_jacobian() {
        let t = iso([0.3, -0.2, 0.5], [0.1, 0.2, 0.3, 1.0]).unwrap();
        let j = inverse_jacobian(&t);
        let ji = inverse_jacobian(&t.inverse());
        let prod = ji * j;
        assert!((prod - Matrix6::identity()).abs().max() < 1e-9);
    }

    #[test]
    fn compose_identity_covariance() {
        let t = iso([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
        let s = Matrix6::identity() * 0.01;
        let sc = compose_cov(&t, &s, &Iso::identity(), &s);
        assert!((sc[(3, 3)] - 0.02).abs() < 1e-12);
    }

    #[test]
    fn bad_covariance_rejected() {
        let mut v = vec![0.0; 36];
        v[0] = -1.0;
        assert!(validate_cov(&v).is_err());
        let mut v2 = vec![0.0; 36];
        for i in 0..6 {
            v2[i * 7] = 1.0;
        }
        v2[1] = 0.5; // 不对称
        assert!(validate_cov(&v2).is_err());
        assert!(validate_cov(&[1.0, 2.0]).is_err());
    }

    #[test]
    fn zero_covariance_is_singular_but_zero_allowed() {
        let z = vec![0.0; 36];
        assert!(validate_cov(&z).is_err());
        assert!(validate_cov(&[]).is_ok());
    }

    #[test]
    fn interpolation_endpoints() {
        let a = iso([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
        let b = iso([1.0, 2.0, 3.0], [0.0, 0.0, 0.30901699, 0.95105652]).unwrap();
        let m = interpolate(&a, &b, 0.5);
        assert!((m.translation.x - 0.5).abs() < 1e-9);
        let a2 = interpolate(&a, &b, 0.0);
        assert!((a2.translation.vector - a.translation.vector).abs().max() < 1e-9);
        let b2 = interpolate(&a, &b, 1.0);
        assert!((b2.translation.vector - b.translation.vector).abs().max() < 1e-9);
    }
}
