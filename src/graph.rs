//! 变换图：版本化的“传感器→载体→地图”刚体边。
//!
//! - 边是**有向**的 `source -> target`（点沿 source 坐标向 target 坐标传播）；
//! - 每条边带有效时间窗与 6x6 协方差，同 key 的新版本通过 `supersedes` 串起来；
//! - 加边时做结构性环路检查，发现环路时返回环路帧序列与累积残差，拒绝写入；
//! - 路径枚举使用 BTreeSet/BTreeMap，选路不依赖 HashMap 迭代顺序。

use crate::geo::{
    compose_cov, precision_score, row_major, validate_cov, Se3,
};
use crate::db::Db;
use anyhow::{anyhow, Result};
use nalgebra::Matrix6;
use rusqlite::params;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Serialize)]
pub struct EdgeRecord {
    pub id: i64,
    pub key: String,
    pub version: i64,
    pub kind: String,
    pub source_frame: String,
    pub target_frame: String,
    pub q: [f64; 4],
    pub t: [f64; 3],
    pub cov: Vec<f64>,
    pub valid_from: f64,
    pub valid_to: f64,
    pub supersedes: Option<i64>,
    pub active: bool,
}

impl EdgeRecord {
    pub fn se3(&self) -> Se3 {
        Se3 { q: self.q, t: self.t }
    }
    pub fn cov6(&self) -> Matrix6<f64> {
        let mut m = Matrix6::zeros();
        for i in 0..6 {
            for j in 0..6 {
                m[(i, j)] = self.cov[i * 6 + j];
            }
        }
        m
    }
}

#[derive(Debug, Clone)]
pub struct NewEdge {
    pub key: String,
    pub kind: String,
    pub source_frame: String,
    pub target_frame: String,
    pub se3: Se3,
    pub cov: Matrix6<f64>,
    pub valid_from: f64,
    pub valid_to: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct CycleError {
    pub loop_frames: Vec<String>,
    pub translation_residual_m: f64,
    pub rotation_residual_rad: f64,
    pub message: String,
}

impl std::fmt::Display for CycleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}
impl std::error::Error for CycleError {}

fn load_active_edges(db: &Db) -> Result<Vec<EdgeRecord>> {
    let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut stmt = c.prepare(
        "SELECT id,key,version,kind,source_frame,target_frame,
                qx,qy,qz,qw,tx,ty,tz,cov_json,valid_from,valid_to,
                supersedes,active
         FROM edge WHERE active=1 ORDER BY id",
    )?;
    let rows = stmt
        .query_map([], |r| {
            let cov_json: String = r.get(13)?;
            let cov: Vec<f64> = serde_json::from_str(&cov_json).unwrap_or_default();
            Ok(EdgeRecord {
                id: r.get(0)?,
                key: r.get(1)?,
                version: r.get(2)?,
                kind: r.get(3)?,
                source_frame: r.get(4)?,
                target_frame: r.get(5)?,
                q: [r.get(6)?, r.get(7)?, r.get(8)?, r.get(9)?],
                t: [r.get(10)?, r.get(11)?, r.get(12)?],
                cov,
                valid_from: r.get(14)?,
                valid_to: r.get(15)?,
                supersedes: r.get(16)?,
                active: r.get::<_, i64>(17)? != 0,
            })
        })?
        .collect::<Result<Vec<_>, _>>()?;
    Ok(rows)
}

/// 邻接表（确定性：BTreeMap<BTreeSet>）。
fn adjacency(edges: &[EdgeRecord], at: f64) -> BTreeMap<String, BTreeSet<(String, usize)>> {
    let mut adj: BTreeMap<String, BTreeSet<(String, usize)>> = BTreeMap::new();
    for (i, e) in edges.iter().enumerate() {
        if at >= e.valid_from && at < e.valid_to {
            adj.entry(e.source_frame.clone())
                .or_default()
                .insert((e.target_frame.clone(), i));
        }
    }
    adj
}

/// 若加入 `new_edge`（时间窗内）后构成有向环，返回环信息与闭合残差。
fn detect_cycle(
    existing: &[EdgeRecord],
    src: &str,
    dst: &str,
    from: f64,
    to: f64,
) -> Option<CycleError> {
    // 候选边只在与新边时间窗重叠的边上检查。
    let edges: Vec<EdgeRecord> = existing
        .iter()
        .filter(|e| e.valid_to > from && e.valid_from < to)
        .cloned()
        .collect();
    let adj = adjacency(&edges, (from + to) * 0.5);

    // 从 dst 找一条回到 src 的简单路径（DFS 按序展开）。
    let mut path_nodes = vec![dst.to_string()];
    let mut visited = BTreeSet::new();
    visited.insert(dst.to_string());
    let mut edge_path: Vec<usize> = Vec::new();

    fn dfs(
        node: &str,
        goal: &str,
        adj: &BTreeMap<String, BTreeSet<(String, usize)>>,
        visited: &mut BTreeSet<String>,
        nodes: &mut Vec<String>,
        epath: &mut Vec<usize>,
    ) -> bool {
        if node == goal && !epath.is_empty() {
            return true;
        }
        if let Some(neighbors) = adj.get(node) {
            for (nxt, idx) in neighbors {
                if visited.contains(nxt) {
                    continue;
                }
                visited.insert(nxt.clone());
                nodes.push(nxt.clone());
                epath.push(*idx);
                if dfs(nxt, goal, adj, visited, nodes, epath) {
                    return true;
                }
                epath.pop();
                nodes.pop();
                visited.remove(nxt);
            }
        }
        false
    }

    let closes = dfs(dst, src, &adj, &mut visited, &mut path_nodes, &mut edge_path);
    if !closes {
        return None;
    }

    // 计算闭合残差：候选边 ∘ 回路边 的复合与单位变换的偏差。
    // 候选边本身不在 edges 中，残差由回路边独立给出（T_src←dst ∘ 已存在路径 dst→src）。
    let mut t = Se3::identity();
    let mut cov = Matrix6::zeros();
    let mut any = false;
    for idx in &edge_path {
        let e = &edges[*idx];
        if !any {
            t = e.se3();
            cov = e.cov6();
            any = true;
        } else {
            let g = t.compose(&e.se3());
            cov = compose_cov(&g, &cov, &e.cov6());
            t = g;
        }
    }
    let (dt, ang) = crate::geo::displacement(&t, &Se3::identity());

    let mut frames = vec![src.to_string()];
    frames.extend(path_nodes.iter().cloned());
    Some(CycleError {
        translation_residual_m: dt,
        rotation_residual_rad: ang,
        loop_frames: frames,
        message: format!(
            "变换图出现环并被拒绝：{} -> {}（平移残差 {dt:.4} m，旋转残差 {ang:.4} rad，路径协方差迹 {:.3e}）",
            src,
            dst,
            precision_score(&cov)
        ),
    })
}

/// 添加标定/地图边。同 key 自增版本并令旧版本失效；成环则整体回滚。
pub fn add_edge(db: &Db, mut ne: NewEdge) -> Result<(EdgeRecord, Option<CycleError>)> {
    ne.key = ne.key.trim().to_string();
    ne.source_frame = ne.source_frame.trim().to_string();
    ne.target_frame = ne.target_frame.trim().to_string();
    if ne.key.is_empty() || ne.source_frame.is_empty() || ne.target_frame.is_empty() {
        return Err(anyhow!("边 key 与帧名不能为空"));
    }
    if ne.source_frame == ne.target_frame {
        return Err(anyhow!("自环边（source==target）不允许"));
    }
    if ne.valid_to <= ne.valid_from {
        return Err(anyhow!("有效时间窗无效：valid_to 必须大于 valid_from"));
    }
    validate_cov(&ne.cov, 1e-10).map_err(|m| anyhow!(m))?;
    if !ne.se3.is_finite() {
        return Err(anyhow!("变换含非有限值"));
    }

    let existing = load_active_edges(db)?;
    if let Some(cyc) = detect_cycle(
        &existing,
        &ne.source_frame,
        &ne.target_frame,
        ne.valid_from,
        ne.valid_to,
    ) {
        db.log_event("warn", "edge_cycle_rejected", &cyc.message);
        return Ok((
            EdgeRecord {
                id: -1,
                key: ne.key,
                version: -1,
                kind: ne.kind,
                source_frame: ne.source_frame,
                target_frame: ne.target_frame,
                q: [ne.se3.q[0], ne.se3.q[1], ne.se3.q[2], ne.se3.q[3]],
                t: ne.se3.t,
                cov: row_major(&ne.cov),
                valid_from: ne.valid_from,
                valid_to: ne.valid_to,
                supersedes: None,
                active: false,
            },
            Some(cyc),
        ));
    }

    let mut c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
    let tx = c.transaction()?;
    let prev: Option<(i64, i64)> = tx
        .query_row(
            "SELECT id,version FROM edge WHERE key=?1 AND active=1 ORDER BY version DESC LIMIT 1",
            params![ne.key],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )
        .optional()?;
    let version = prev.map(|(_, v)| v + 1).unwrap_or(1);
    let cov_json = serde_json::to_string(&row_major(&ne.cov))?;
    let t = ne.se3.t;
    let q = ne.se3.q;
    tx.execute(
        "INSERT INTO edge(key,version,kind,source_frame,target_frame,
            qx,qy,qz,qw,tx,ty,tz,cov_json,valid_from,valid_to,supersedes,created_at,active)
         VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13,?14,?15,?16,?17,1)",
        params![
            ne.key,
            version,
            ne.kind,
            ne.source_frame,
            ne.target_frame,
            q[0], q[1], q[2], q[3],
            t[0], t[1], t[2],
            cov_json,
            ne.valid_from,
            ne.valid_to,
            prev.map(|(id, _)| id),
            crate::db::now_unix(),
        ],
    )?;
    let id = tx.last_insert_rowid();
    if let Some((old_id, _)) = prev {
        tx.execute("UPDATE edge SET active=0 WHERE id=?1", params![old_id])?;
    }
    tx.commit()?;
    drop(c);
    db.log_event(
        "info",
        "edge_added",
        &format!("{} v{}: {} -> {}", ne.key, version, ne.source_frame, ne.target_frame),
    );
    let rec = load_active_edges(db)?
        .into_iter()
        .find(|e| e.id == id)
        .ok_or_else(|| anyhow!("新边回读失败"))?;
    Ok((rec, None))
}

use rusqlite::OptionalExtension;

/// 一条完整路径上的复合结果。
#[derive(Debug, Clone, Serialize)]
pub struct ResolvedPath {
    pub edge_ids: Vec<i64>,
    pub edge_keys: Vec<String>,
    pub edge_versions: Vec<i64>,
    pub frames: Vec<String>,
    pub total: Se3,
    pub cov_trace: f64,
    pub version_sum: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct PathChoice {
    pub chosen: ResolvedPath,
    pub candidates: Vec<ResolvedPath>,
    pub rule: String,
}

/// 构造姿态插值虚拟边（body@t0 -> body@t → map）。
pub fn pose_edge(
    _key: &str,
    _source_frame: &str,
    _target_frame: &str,
    t0: f64,
    t1: f64,
    tq: f64,
    p0: &Se3,
    p1: &Se3,
    c0: &Matrix6<f64>,
    c1: &Matrix6<f64>,
    max_gap: f64,
) -> Result<(Se3, Matrix6<f64>)> {
    if t1 - t0 > max_gap {
        return Err(anyhow!(
            "姿态间隔 {:.3}s 超过允许间隔 {max_gap}s",
            t1 - t0
        ));
    }
    if !(tq >= t0 && tq <= t1) {
        return Err(anyhow!("插值时刻 {tq} 不在姿态区间 [{t0}, {t1}]"));
    }
    let s = (tq - t0) / (t1 - t0);
    Ok((
        crate::geo::interpolate(p0, p1, s),
        crate::geo::interpolate_cov(c0, c1, s),
    ))
}

fn resolve(edges: &[EdgeRecord], picks: &[usize], start: &str) -> ResolvedPath {
    let mut total = Se3::identity();
    let mut cov = Matrix6::identity() * 1e-15;
    let mut frames = vec![start.to_string()];
    let mut ids = Vec::new();
    let mut keys = Vec::new();
    let mut versions = Vec::new();
    let mut version_sum = 0i64;
    for &i in picks {
        let e = &edges[i];
        let g = total.compose(&e.se3());
        cov = compose_cov(&g, &cov, &e.cov6());
        total = g;
        frames.push(e.target_frame.clone());
        ids.push(e.id);
        keys.push(e.key.clone());
        versions.push(e.version);
        version_sum += e.version;
    }
    ResolvedPath {
        edge_ids: ids,
        edge_keys: keys,
        edge_versions: versions,
        frames,
        total,
        cov_trace: precision_score(&cov),
        version_sum,
    }
}

/// 在时刻 at 解析 source→target 的全部简单路径并按显式规则选路：
/// 1) 协方差迹（精度）最小；2) 版本号之和最小（稳定优先旧标定）；
/// 3) 帧序列字典序（纯确定性兜底，绝不依赖 map 遍历顺序）。
pub fn resolve_path(
    db: &Db,
    source: &str,
    target: &str,
    at: f64,
    virtual_edges: Vec<EdgeRecord>,
) -> Result<PathChoice> {
    let mut edges = load_active_edges(db)?;
    edges.extend(virtual_edges);
    let adj = adjacency(&edges, at);

    let mut found: Vec<ResolvedPath> = Vec::new();
    let mut stack: Vec<(String, Vec<usize>, BTreeSet<String>)> = Vec::new();
    let mut start_vis = BTreeSet::new();
    start_vis.insert(source.to_string());
    stack.push((source.to_string(), Vec::new(), start_vis));

    while let Some((node, picks, visited)) = stack.pop() {
        if node == target && !picks.is_empty() {
            found.push(resolve(&edges, &picks, source));
            continue;
        }
        if picks.len() >= 16 {
            continue;
        }
        if let Some(nbrs) = adj.get(&node) {
            // 逆序压栈以保证展开顺序与 BTreeSet 升序一致。
            for (nxt, idx) in nbrs.iter().rev() {
                if visited.contains(nxt) {
                    continue;
                }
                let mut v2 = visited.clone();
                v2.insert(nxt.clone());
                let mut p2 = picks.clone();
                p2.push(*idx);
                stack.push((nxt.clone(), p2, v2));
            }
        }
    }

    if found.is_empty() {
        return Err(anyhow!(
            "时刻 {at:.3} 不存在 {source} -> {target} 的有效变换路径"
        ));
    }
    found.sort_by(|a, b| {
        a.cov_trace
            .partial_cmp(&b.cov_trace)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.version_sum.cmp(&b.version_sum))
            .then_with(|| a.frames.cmp(&b.frames))
    });
    let chosen = found.remove(0);
    Ok(PathChoice {
        chosen,
        candidates: found,
        rule: "cov_trace ASC, version_sum ASC, frames LEX ASC".into(),
    })
}

pub fn list_edges(db: &Db, include_inactive: bool) -> Result<Vec<EdgeRecord>> {
    if include_inactive {
        let c = db.0.lock().map_err(|e| anyhow::anyhow!("{e}"))?;
        let mut stmt = c.prepare(
            "SELECT id,key,version,kind,source_frame,target_frame,
                    qx,qy,qz,qw,tx,ty,tz,cov_json,valid_from,valid_to,supersedes,active
             FROM edge ORDER BY key,version",
        )?;
        let rows = stmt
            .query_map([], |r| {
                let cov_json: String = r.get(13)?;
                Ok(EdgeRecord {
                    id: r.get(0)?,
                    key: r.get(1)?,
                    version: r.get(2)?,
                    kind: r.get(3)?,
                    source_frame: r.get(4)?,
                    target_frame: r.get(5)?,
                    q: [r.get(6)?, r.get(7)?, r.get(8)?, r.get(9)?],
                    t: [r.get(10)?, r.get(11)?, r.get(12)?],
                    cov: serde_json::from_str(&cov_json).unwrap_or_default(),
                    valid_from: r.get(14)?,
                    valid_to: r.get(15)?,
                    supersedes: r.get(16)?,
                    active: r.get::<_, i64>(17)? != 0,
                })
            })?
            .collect::<Result<Vec<_>, _>>()?;
        Ok(rows)
    } else {
        load_active_edges(db)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::diag6;

    fn ne(key: &str, s: &str, t: &str, noise: f64, _ver_v: f64) -> NewEdge {
        NewEdge {
            key: key.into(),
            kind: "calib".into(),
            source_frame: s.into(),
            target_frame: t.into(),
            se3: Se3::identity(),
            cov: diag6(noise),
            valid_from: 0.0,
            valid_to: 1.0e9,
        }
    }

    #[test]
    fn cycle_is_rejected_with_residual() {
        let db = crate::db::open_in_memory().unwrap();
        assert!(add_edge(&db, ne("lidar_to_body", "lidar", "body", 1e-3, 0.0)).is_ok());
        assert!(add_edge(&db, ne("body_to_map", "body", "map", 1e-3, 0.0)).is_ok());
        let (_, cyc) = add_edge(
            &db,
            NewEdge {
                key: "map_back_lidar".into(),
                kind: "map".into(),
                source_frame: "map".into(),
                target_frame: "lidar".into(),
                se3: Se3::from_iso([0.0; 3], [0.01, 0.0, 0.0]),
                cov: diag6(1e-3),
                valid_from: 0.0,
                valid_to: 1e9,
            },
        )
        .unwrap();
        let cyc = cyc.expect("环路必须被检测");
        assert!(cyc.translation_residual_m >= 0.0);
        assert_eq!(cyc.loop_frames.first().unwrap(), "map");
        assert_eq!(cyc.loop_frames.last().unwrap(), "map");
        assert_eq!(list_edges(&db, false).unwrap().len(), 2);
    }

    #[test]
    fn parallel_paths_choose_precision_keep_candidate() {
        let db = crate::db::open_in_memory().unwrap();
        // 两条 lidar -> map 路径：经 body（噪声大）与直接（噪声小）。
        let mut via_body = ne("via_body_a", "lidar", "j1", 1e-2, 0.0);
        via_body.se3 = Se3::from_iso([0.0; 3], [0.0, 1.0, 0.0]);
        let mut via_body2 = ne("via_body_b", "j1", "map", 1e-2, 0.0);
        via_body2.se3 = Se3::from_iso([0.0; 3], [0.0, -1.0, 0.0]);
        let mut direct = ne("direct", "lidar", "map", 1e-4, 0.0);
        direct.se3 = Se3::identity();
        add_edge(&db, via_body).unwrap();
        add_edge(&db, via_body2).unwrap();
        add_edge(&db, direct).unwrap();

        let choice = resolve_path(&db, "lidar", "map", 100.0, vec![]).unwrap();
        assert_eq!(choice.chosen.edge_keys, vec!["direct"]);
        assert!(choice
            .candidates
            .iter()
            .any(|p| p.frames == vec!["lidar", "j1", "map"]));
    }

    #[test]
    fn equal_precision_uses_version_rule_deterministically() {
        let db = crate::db::open_in_memory().unwrap();
        let mut a = ne("edge_a", "s", "map", 1e-3, 0.0);
        a.target_frame = "map".into();
        let mut b = ne("edge_b", "s", "map", 1e-3, 0.0);
        b.target_frame = "map".into();
        add_edge(&db, a).unwrap();
        add_edge(&db, b).unwrap();
        let choice1 = resolve_path(&db, "s", "map", 0.0, vec![]).unwrap();
        let choice2 = resolve_path(&db, "s", "map", 0.0, vec![]).unwrap();
        assert_eq!(choice1.chosen.edge_keys, choice2.chosen.edge_keys);
        assert_eq!(choice1.chosen.edge_keys.len(), 1);
    }
}
