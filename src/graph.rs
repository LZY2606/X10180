//! 刚体变换图：时间窗多版本边、环路检测与残差、确定性多路径
//! 枚举、按精度/版本选路、SE(3) 链式误差传播。

use crate::se3::{compose_cov, invert_cov, iso, Iso};
use nalgebra::Matrix6;
use serde::Serialize;
use std::collections::BTreeSet;

pub const LOOP_TOL_TRANS: f64 = 0.05;
pub const LOOP_TOL_ROT: f64 = 0.01;

#[derive(Debug, Clone, Serialize)]
pub struct Hop {
    pub edge_id: i64,
    pub version: i64,
    pub from: String,
    pub to: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub precision: f64,
    pub sigma_after: Vec<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Path {
    pub hops: Vec<Hop>,
    pub precision_sum: f64,
    pub version_sum: i64,
    pub edge_id_sum: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct LoopReport {
    pub frames: Vec<String>,
    pub residual_trans: f64,
    pub residual_rot: f64,
    pub tolerance_trans: f64,
    pub tolerance_rot: f64,
    pub accepted: bool,
    pub compared_edge: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct AddReport {
    pub edge_id: i64,
    pub version: i64,
    pub loops: Vec<LoopReport>,
    pub rejected: bool,
    pub reason: Option<String>,
}

#[derive(Debug, Clone)]
pub struct GraphEdge {
    pub id: i64,
    pub version: i64,
    pub parent: String,
    pub child: String,
    pub transform: Iso,
    pub cov: Matrix6<f64>,
    pub precision: f64,
    pub valid_from: f64,
    pub valid_to: f64,
    pub inserted_at: f64,
}

impl GraphEdge {
    fn active(&self, t: f64) -> bool {
        self.valid_from <= t && t <= self.valid_to
    }

    fn pair(&self) -> (String, String) {
        pair_key(&self.parent, &self.child)
    }
}

#[derive(Clone)]
struct Link {
    edge_id: i64,
    version: i64,
    from: String,
    to: String,
    tf: Iso,
    cov: Matrix6<f64>,
    precision: f64,
}

struct State {
    chain: Vec<Link>,
    tf: Iso,
    cov: Matrix6<f64>,
    visited: BTreeSet<String>,
}

#[derive(Default)]
pub struct FrameGraph {
    edges: Vec<GraphEdge>,
}

impl FrameGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn load(&mut self, edges: Vec<GraphEdge>) {
        self.edges = edges;
    }

    pub fn edges(&self) -> &[GraphEdge] {
        &self.edges
    }

    pub fn add(&mut self, e: GraphEdge) -> AddReport {
        let mut report = AddReport {
            edge_id: e.id,
            version: e.version,
            loops: Vec::new(),
            rejected: false,
            reason: None,
        };
        for other in &self.edges {
            if e.pair() == other.pair() && overlap(&e, other) {
                let lp = loop_report(&e, other);
                if !lp.accepted {
                    report.rejected = true;
                    let msg = format!(
                        "与边#{} 闭环残差超限(平移 {:.4}m>{:.3}, 旋转 {:.5}rad>{:.4})",
                        other.id,
                        lp.residual_trans,
                        LOOP_TOL_TRANS,
                        lp.residual_rot,
                        LOOP_TOL_ROT
                    );
                    match report.reason.as_mut() {
                        Some(r) => {
                            r.push_str("; ");
                            r.push_str(&msg);
                        }
                        None => report.reason = Some(msg),
                    }
                }
                report.loops.push(lp);
            }
        }
        if !report.rejected {
            self.edges.push(e);
        }
        report
    }

    fn links_at(&self, frame: &str, t: f64) -> Vec<Link> {
        let mut out = Vec::new();
        for e in &self.edges {
            if !e.active(t) {
                continue;
            }
            if e.child == frame {
                out.push(Link {
                    edge_id: e.id,
                    version: e.version,
                    from: e.child.clone(),
                    to: e.parent.clone(),
                    tf: e.transform,
                    cov: e.cov,
                    precision: e.precision,
                });
            } else if e.parent == frame {
                out.push(Link {
                    edge_id: e.id,
                    version: e.version,
                    from: e.parent.clone(),
                    to: e.child.clone(),
                    tf: e.transform.inverse(),
                    cov: invert_cov(&e.transform, &e.cov),
                    precision: e.precision,
                });
            }
        }
        out.sort_by(|a, b| (a.edge_id, a.to.clone()).cmp(&(b.edge_id, b.to.clone())));
        out
    }

    /// 枚举 from -> to 的所有无重复框简单路径（<=8 跳），按选路规则排序。
    pub fn find_paths(&self, from: &str, to: &str, t: f64) -> Vec<Path> {
        if from == to {
            return vec![Path {
                hops: Vec::new(),
                precision_sum: 0.0,
                version_sum: 0,
                edge_id_sum: 0,
            }];
        }
        let mut results: Vec<Path> = Vec::new();
        let mut stack: Vec<State> = Vec::new();
        for l in self.links_at(from, t) {
            let mut visited = BTreeSet::new();
            visited.insert(from.to_string());
            visited.insert(l.to.clone());
            stack.push(State {
                tf: l.tf,
                cov: l.cov,
                chain: vec![l],
                visited,
            });
        }
        while let Some(st) = stack.pop() {
            let node = st.chain.last().unwrap().to.clone();
            if node == to {
                results.push(build_path(&st.chain));
                continue;
            }
            if st.chain.len() >= 8 {
                continue;
            }
            let mut nexts = self.links_at(&node, t);
            nexts.retain(|l| !st.visited.contains(&l.to));
            nexts.sort_by(|a, b| (a.edge_id, a.to.clone()).cmp(&(b.edge_id, b.to.clone())));
            // 逆序压栈，使最小 id 先弹出（结果顺序仍最终显式排序，双保险）。
            for l in nexts.into_iter().rev() {
                let ntf = st.tf * l.tf;
                let ncov = compose_cov(&st.tf, &st.cov, &l.tf, &l.cov);
                let mut visited = st.visited.clone();
                visited.insert(l.to.clone());
                let mut chain = st.chain.clone();
                chain.push(l);
                stack.push(State {
                    chain,
                    tf: ntf,
                    cov: ncov,
                    visited,
                });
            }
        }
        results.sort_by(path_rank_cmp);
        results.dedup_by(|a, b| hop_ids(a) == hop_ids(b));
        results
    }
}

fn build_path(links: &[Link]) -> Path {
    let mut hops = Vec::new();
    let mut acc = Matrix6::zeros();
    let mut running = Iso::identity();
    for (i, l) in links.iter().enumerate() {
        acc = if i == 0 {
            l.cov
        } else {
            compose_cov(&running, &acc, &l.tf, &l.cov)
        };
        running = running * l.tf;
        let (t, q) = crate::se3::params(&l.tf);
        hops.push(Hop {
            edge_id: l.edge_id,
            version: l.version,
            from: l.from.clone(),
            to: l.to.clone(),
            tx: t[0],
            ty: t[1],
            tz: t[2],
            qx: q[0],
            qy: q[1],
            qz: q[2],
            qw: q[3],
            precision: l.precision,
            sigma_after: acc.iter().copied().collect(),
        });
    }
    Path {
        hops,
        precision_sum: links.iter().map(|l| l.precision).sum::<f64>(),
        version_sum: links.iter().map(|l| l.version).sum::<i64>(),
        edge_id_sum: links.iter().map(|l| l.edge_id).sum::<i64>(),
    }
}

fn hop_ids(p: &Path) -> Vec<i64> {
    p.hops.iter().map(|h| h.edge_id).collect()
}

fn path_rank_cmp(a: &Path, b: &Path) -> std::cmp::Ordering {
    a.precision_sum
        .partial_cmp(&b.precision_sum)
        .unwrap_or(std::cmp::Ordering::Equal)
        .then(b.version_sum.cmp(&a.version_sum))
        .then(a.edge_id_sum.cmp(&b.edge_id_sum))
        .then_with(|| hop_ids(a).cmp(&hop_ids(b)))
}

fn pair_key(a: &str, b: &str) -> (String, String) {
    if a <= b {
        (a.to_string(), b.to_string())
    } else {
        (b.to_string(), a.to_string())
    }
}

fn overlap(a: &GraphEdge, b: &GraphEdge) -> bool {
    a.valid_from <= b.valid_to && b.valid_from <= a.valid_to
}

fn loop_report(new: &GraphEdge, old: &GraphEdge) -> LoopReport {
    let a = new.transform;
    let b = if old.child == new.child && old.parent == new.parent {
        old.transform
    } else {
        old.transform.inverse()
    };
    let rel = a.inverse() * b;
    let dt = rel.translation.vector.norm();
    let ang = rel.rotation.angle().abs();
    LoopReport {
        frames: vec![new.child.clone(), new.parent.clone()],
        residual_trans: dt,
        residual_rot: ang,
        tolerance_trans: LOOP_TOL_TRANS,
        tolerance_rot: LOOP_TOL_ROT,
        accepted: dt <= LOOP_TOL_TRANS && ang <= LOOP_TOL_ROT,
        compared_edge: old.id,
    }
}

pub fn make_edge(
    id: i64,
    version: i64,
    parent: &str,
    child: &str,
    t: [f64; 3],
    q: [f64; 4],
    cov: Matrix6<f64>,
    precision: Option<f64>,
    valid_from: f64,
    valid_to: f64,
    inserted_at: f64,
) -> Result<GraphEdge, String> {
    if parent == child {
        return Err("变换的 parent 与 child 不能相同".into());
    }
    if !valid_to.is_finite() || !valid_from.is_finite() || valid_to < valid_from {
        return Err("有效时间区间非法".into());
    }
    let transform = iso(t, q)?;
    let precision = precision.unwrap_or_else(|| {
        let tr = cov.trace();
        if tr > 0.0 {
            tr.sqrt()
        } else {
            1.0
        }
    });
    Ok(GraphEdge {
        id,
        version,
        parent: parent.to_string(),
        child: child.to_string(),
        transform,
        cov,
        precision,
        valid_from,
        valid_to,
        inserted_at,
    })
}
