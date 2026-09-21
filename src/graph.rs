//! 变换图：帧为节点，带版本刚体变换为有向边。
//! - 加边前做环路检测，拒绝时给出环路与累积残差。
//! - 多路径选路：按（累积协方差迹升序, 最小边版本降序, 帧序列字典序）确定性排序，
//!   未选路径保留为候选；全程使用 BTreeMap/显式排序，不依赖 map 遍历顺序。

use crate::math::{Mat6, SE3};
use std::collections::BTreeMap;

pub const MAX_PATH_DEPTH: usize = 8;
pub const MAX_PATHS: usize = 64;

#[derive(Clone, Debug)]
pub struct Edge {
    pub src: String,
    pub dst: String,
    pub version: i64,
    pub tf: SE3,
    pub cov: Mat6,
    pub valid_from: f64,
    pub valid_to: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Graph {
    /// (src, dst) -> 按版本升序的边列表。BTreeMap 保证遍历确定。
    pub edges: BTreeMap<(String, String), Vec<Edge>>,
}

#[derive(Clone, Debug)]
pub struct Path {
    pub frames: Vec<String>,
    pub edges: Vec<Edge>,
    pub composed: SE3,
    pub cov: Mat6,
}

impl Path {
    /// 选路排序键：精度（协方差迹）优先，其次最小边版本（新者优先），最后帧序列字典序。
    fn key(&self) -> (OrdF64, std::cmp::Reverse<i64>, Vec<String>) {
        let min_version = self.edges.iter().map(|e| e.version).min().unwrap_or(0);
        (OrdF64(self.cov.trace()), std::cmp::Reverse(min_version), self.frames.clone())
    }
}

/// 避免引入外部依赖的最小全序浮点包装。
#[derive(Clone, Copy, PartialEq, PartialOrd)]
struct OrdF64(f64);
impl Eq for OrdF64 {}
impl Ord for OrdF64 {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.0.partial_cmp(&o.0).unwrap_or(std::cmp::Ordering::Equal)
    }
}

#[derive(Clone, Debug)]
pub struct CycleError {
    pub cycle: Vec<String>,
    pub residual: f64,
}

impl std::fmt::Display for CycleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "变换图存在环路 {}，累积残差 {:.6e}", self.cycle.join(" -> "), self.residual)
    }
}

impl Graph {
    pub fn add_edge(&mut self, e: Edge) {
        let list = self.edges.entry((e.src.clone(), e.dst.clone())).or_default();
        list.push(e);
        list.sort_by_key(|x| x.version);
    }

    /// 当前节点按字典序排列的出边（每对节点取最新版本），保证 DFS 顺序确定。
    fn sorted_out_edges(&self, node: &str) -> Vec<Edge> {
        self.edges
            .iter()
            .filter(|((s, _), _)| s == node)
            .filter_map(|(_, v)| v.last().cloned())
            .collect()
    }

    /// 枚举 from→to 的全部简单路径（深度与数量受限），按选路规则排序后返回。
    pub fn find_paths(&self, from: &str, to: &str) -> Vec<Path> {
        let mut out = Vec::new();
        let mut frames = vec![from.to_string()];
        let mut edges: Vec<Edge> = Vec::new();
        self.dfs(from, to, &mut frames, &mut edges, &mut out);
        out.sort_by(|a, b| a.key().cmp(&b.key()));
        out
    }

    fn dfs(
        &self,
        node: &str,
        to: &str,
        frames: &mut Vec<String>,
        edges: &mut Vec<Edge>,
        out: &mut Vec<Path>,
    ) {
        if out.len() >= MAX_PATHS {
            return;
        }
        if node == to && !edges.is_empty() {
            out.push(Self::compose_path(frames.clone(), edges.clone()));
            return;
        }
        if edges.len() >= MAX_PATH_DEPTH {
            return;
        }
        for e in self.sorted_out_edges(node) {
            if frames.contains(&e.dst) {
                continue;
            }
            frames.push(e.dst.clone());
            edges.push(e.clone());
            self.dfs(&e.dst.clone(), to, frames, edges, out);
            edges.pop();
            frames.pop();
        }
    }

    fn compose_path(frames: Vec<String>, edges: Vec<Edge>) -> Path {
        let mut composed = SE3::identity();
        let mut cov = Mat6::zero();
        for e in &edges {
            cov = Mat6::propagate(&composed, &cov, &e.cov);
            composed = composed.compose(e.tf);
        }
        Path { frames, edges, composed, cov }
    }

    /// 加边前的环路检测：若 dst→…→src 已存在路径，则新边将成环。
    /// 返回环路与累积残差（整环组合与恒等变换的偏差）。
    pub fn check_cycle(&self, new_edge: &Edge) -> Result<(), CycleError> {
        if new_edge.src == new_edge.dst {
            return Err(CycleError {
                cycle: vec![new_edge.src.clone(), new_edge.dst.clone()],
                residual: new_edge.tf.residual_to_identity(),
            });
        }
        let mut paths = self.find_paths(&new_edge.dst, &new_edge.src);
        if let Some(p) = paths.drain(..).next() {
            let mut cycle = p.frames.clone();
            cycle.push(new_edge.dst.clone());
            // 环 = 已有路径(dst→…→src) 接上 新边(src→dst)
            let mut ring = SE3::identity();
            for e in &p.edges {
                ring = ring.compose(e.tf);
            }
            ring = ring.compose(new_edge.tf);
            return Err(CycleError { cycle, residual: ring.residual_to_identity() });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Quat, Vec3};

    fn edge(src: &str, dst: &str, version: i64, tx: f64, cov_trace: f64) -> Edge {
        Edge {
            src: src.into(),
            dst: dst.into(),
            version,
            tf: SE3::new(Quat::identity(), Vec3::new(tx, 0.0, 0.0)),
            cov: Mat6::from_diag([cov_trace / 6.0; 6]),
            valid_from: f64::NEG_INFINITY,
            valid_to: f64::INFINITY,
        }
    }

    #[test]
    fn cycle_rejected_with_path_and_residual() {
        let mut g = Graph::default();
        g.add_edge(edge("a", "b", 1, 1.0, 0.1));
        g.add_edge(edge("b", "c", 1, 2.0, 0.1));
        let err = g.check_cycle(&edge("c", "a", 1, -3.5, 0.1)).unwrap_err();
        assert_eq!(err.cycle, vec!["a", "b", "c", "a"]);
        assert!((err.residual - 0.5).abs() < 1e-9, "residual={}", err.residual);
    }

    #[test]
    fn parallel_paths_deterministic_choice_and_candidates() {
        let mut g = Graph::default();
        g.add_edge(edge("s", "m", 1, 0.0, 0.6)); // 直达，精度差
        g.add_edge(edge("s", "b", 1, 0.0, 0.1));
        g.add_edge(edge("b", "m", 1, 0.0, 0.1)); // 经由 b，精度好
        let paths = g.find_paths("s", "m");
        assert_eq!(paths.len(), 2);
        assert_eq!(paths[0].frames, vec!["s", "b", "m"]);
        assert_eq!(paths[1].frames, vec!["s", "m"], "未选路径保留为候选");
    }

    #[test]
    fn tie_broken_by_version_then_lexicographic() {
        let mut g = Graph::default();
        g.add_edge(edge("s", "x", 1, 0.0, 0.1));
        g.add_edge(edge("x", "m", 1, 0.0, 0.1));
        g.add_edge(edge("s", "y", 2, 0.0, 0.1));
        g.add_edge(edge("y", "m", 2, 0.0, 0.1));
        let paths = g.find_paths("s", "m");
        assert_eq!(paths.len(), 2);
        assert_eq!(paths[0].frames, vec!["s", "y", "m"], "版本新者优先");
        // 完全并列（同精度同版本）时按帧序列字典序
        let mut g2 = Graph::default();
        g2.add_edge(edge("s", "x", 1, 0.0, 0.1));
        g2.add_edge(edge("x", "m", 1, 0.0, 0.1));
        g2.add_edge(edge("s", "y", 1, 0.0, 0.1));
        g2.add_edge(edge("y", "m", 1, 0.0, 0.1));
        let p2 = g2.find_paths("s", "m");
        assert_eq!(p2[0].frames, vec!["s", "x", "m"]);
    }
}
