//! Time-versioned rigid transform graph.
//!
//! * Edges are directed rigid transforms `source -> target` with a validity
//!   window and a 6x6 covariance.
//! * Updating an edge creates a new version; old versions stay auditable.
//! * Adding an edge whose validity window closes a directed cycle is rejected
//!   with the cycle path and the accumulated loop residual.
//! * Routing enumerates every simple path, scores them by explicit precision
//!   (trace of propagated covariance) plus deterministic version rules, and
//!   records every non-winning route as a candidate.  No HashMap iteration
//!   order is ever observed for selection.

use crate::math::{vnorm, Cov6, Iso, Quat};
use crate::model::AppError;
use rusqlite::{params, Connection};
use serde::Serialize;

#[derive(Clone, Debug)]
pub struct Edge {
    pub id: i64,
    pub source: String,
    pub target: String,
    pub version: i64,
    pub valid_from: Option<f64>,
    pub valid_to: Option<f64>,
    pub iso: Iso,
    pub cov: Cov6,
    pub origin: String,
    pub created_at: f64,
}

pub fn row_to_edge(r: &rusqlite::Row<'_>) -> rusqlite::Result<Edge> {
    let cov_str: String = r.get("cov_json")?;
    let cov: [[f64; 6]; 6] =
        serde_json::from_str(&cov_str).unwrap_or_else(|_| [[0.0; 6]; 6]);
    Ok(Edge {
        id: r.get("id")?,
        source: r.get("source")?,
        target: r.get("target")?,
        version: r.get("version")?,
        valid_from: r.get("valid_from")?,
        valid_to: r.get("valid_to")?,
        iso: Iso::new(
            Quat([
                r.get("qw")?,
                r.get("qx")?,
                r.get("qy")?,
                r.get("qz")?,
            ]),
            [r.get("tx")?, r.get("ty")?, r.get("tz")?],
        ),
        cov: Cov6(cov),
        origin: r.get("origin")?,
        created_at: r.get("created_at")?,
    })
}

pub fn edge_valid_at(e: &Edge, t: f64) -> bool {
    e.valid_from.map_or(true, |a| t >= a)
        && e.valid_to.map_or(true, |b| t < b)
}

/// Do two half-open validity windows overlap?  `None` means unbounded.
pub fn windows_overlap(
    a0: Option<f64>,
    a1: Option<f64>,
    b0: Option<f64>,
    b1: Option<f64>,
) -> bool {
    let lo = match (a0, b0) {
        (None, None) => f64::NEG_INFINITY,
        (Some(x), None) | (None, Some(x)) => x,
        (Some(x), Some(y)) => x.max(y),
    };
    let hi = match (a1, b1) {
        (None, None) => f64::INFINITY,
        (Some(x), None) | (None, Some(x)) => x,
        (Some(x), Some(y)) => x.min(y),
    };
    lo < hi
}

pub fn all_edges(c: &Connection) -> rusqlite::Result<Vec<Edge>> {
    let mut stmt = c.prepare(
        "SELECT * FROM edges ORDER BY source, target, version",
    )?;
    let rows = stmt.query_map([], row_to_edge)?;
    let mut v = Vec::new();
    for x in rows {
        v.push(x?);
    }
    Ok(v)
}

/// Insert (or supersede) a static transform edge.  A new edge with the same
/// `(source, target)` gets `version = max(version)+1`; the previous version
/// is recorded in `supersedes`.  Returns the new edge id.
pub fn upsert_edge(
    c: &Connection,
    source: &str,
    target: &str,
    valid_from: Option<f64>,
    valid_to: Option<f64>,
    iso: &Iso,
    cov: &Cov6,
    origin: &str,
    now: f64,
) -> Result<i64, AppError> {
    if !cov.is_positive_definite() {
        return Err(AppError::singular(format!(
            "covariance of edge {source}->{target} is not positive definite"
        )));
    }
    let prev: Option<(i64, i64)> = c
        .query_row(
            "SELECT id, version FROM edges WHERE source=?1 AND target=?2
             ORDER BY version DESC LIMIT 1",
            params![source, target],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )
        .ok();
    let version = prev.map(|(_, v)| v + 1).unwrap_or(1);
    let supersedes = prev.map(|(id, _)| id);

    // Cycle check against edges active anywhere in the new validity window.
    let active = edges_overlapping(c, valid_from, valid_to).map_err(db)?;
    if let Some(cycle) = find_cycle(&active, source, target) {
        return Err(AppError::cycle(format!(
            "edge {source}->{target} closes a cycle [residual t={:.4}m r={:.4e}rad]: {}",
            cycle.residual_translation_m,
            cycle.residual_rotation_rad,
            cycle.frames.join(" -> ")
        )));
    }

    c.execute(
        "INSERT OR IGNORE INTO frames(name, kind) VALUES(?1,'sensor'),
         (?2,'frame')",
        params![source, target],
    )
    .map_err(db)?;
    c.execute(
        "INSERT INTO edges
         (source,target,version,supersedes,valid_from,valid_to,
          qw,qx,qy,qz,tx,ty,tz,cov_json,origin,created_at)
         VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13,?14,?15,?16)",
        rusqlite::params![
            source,
            target,
            version,
            supersedes,
            valid_from,
            valid_to,
            iso.q.0[0],
            iso.q.0[1],
            iso.q.0[2],
            iso.q.0[3],
            iso.t[0],
            iso.t[1],
            iso.t[2],
            serde_json::to_string(&cov.0).unwrap(),
            origin,
            now,
        ],
    )
    .map_err(db)?;
    Ok(c.last_insert_rowid())
}

fn db(e: rusqlite::Error) -> AppError {
    AppError::new("db_error", e.to_string())
}

fn edges_overlapping(
    c: &Connection,
    from: Option<f64>,
    to: Option<f64>,
) -> rusqlite::Result<Vec<Edge>> {
    let mut stmt =
        c.prepare("SELECT * FROM edges ORDER BY source,target,version")?;
    let rows = stmt.query_map([], row_to_edge)?;
    let mut out = Vec::new();
    for e in rows {
        let e = e?;
        if windows_overlap(e.valid_from, e.valid_to, from, to) {
            out.push(e);
        }
    }
    Ok(out)
}

#[derive(Clone, Debug, Serialize)]
pub struct CycleReport {
    /// Frames around the loop, starting and ending at `source`.
    pub frames: Vec<String>,
    pub residual_translation_m: f64,
    /// Residual rotation magnitude in radians (angle of the composed quat).
    pub residual_rotation_rad: f64,
}

/// Deterministic DFS for an existing path `from -> to`.  Neighbours are
/// visited in lexicographic frame order so reports never depend on map order.
fn find_path(edges: &[Edge], from: &str, to: &str) -> Option<Vec<String>> {
    let mut adj: std::collections::BTreeMap<&str, Vec<&str>> =
        std::collections::BTreeMap::new();
    for e in edges {
        adj.entry(e.source.as_str()).or_default().push(&e.target);
    }
    for v in adj.values_mut() {
        v.sort();
        v.dedup();
    }
    let mut stack: Vec<(&str, Vec<&str>)> = vec![(from, vec![from])];
    while let Some((node, path)) = stack.pop() {
        if node == to && path.len() > 1 {
            return Some(path.into_iter().map(String::from).collect());
        }
        if let Some(nexts) = adj.get(node) {
            for nx in nexts.iter().rev() {
                if !path.contains(nx) {
                    let mut p = path.clone();
                    p.push(nx);
                    stack.push((nx, p));
                }
            }
        }
    }
    None
}

pub fn find_cycle(edges: &[Edge], new_source: &str, new_target: &str) -> Option<CycleReport> {
    // The new edge closes a cycle exactly when target is already reachable
    // from source through existing edges.
    let path = find_path(edges, new_target, new_source)?;
    // Frame sequence of the full loop, beginning at new_source.
    let mut frames = vec![new_source.to_string()];
    frames.extend(path.iter().cloned());

    let mut total = Iso::identity();
    // Walk new_source -> new_target (the candidate edge is identity for the
    // residual's relative part; we only compose existing chain edges here)
    // then new_target -> ... -> new_source through stored edges.
    for w in path.windows(2) {
        let e = edges
            .iter()
            .filter(|e| e.source == w[0] && e.target == w[1])
            .max_by(|a, b| a.version.cmp(&b.version))
            .expect("path edge must exist");
        total = total.compose(&e.iso);
    }
    // `total` now maps new_source back onto itself through existing edges
    // (new_target chain).  Loop residual is measured relative to identity.
    let angle = {
        let w = total.q.0[0].clamp(-1.0, 1.0);
        2.0 * w.abs().min(1.0).acos()
    };
    Some(CycleReport {
        frames,
        residual_translation_m: vnorm(total.t),
        residual_rotation_rad: angle,
    })
}

/// One traversable directed arc at routing time.
#[derive(Clone, Debug)]
pub struct RouteArc {
    pub source: String,
    pub target: String,
    pub iso: Iso,
    pub cov: Cov6,
    /// Stable identity for audit, e.g. "edge#7@v2" or "gnss(interp t=12.3)".
    pub arc_id: String,
    /// Larger means preferred by the explicit version rule on exact cost ties.
    pub version_rank: i64,
}

#[derive(Clone, Debug, Serialize)]
pub struct RouteStep {
    pub from: String,
    pub to: String,
    pub arc_id: String,
    pub translation_m: [f64; 3],
    pub quat_wxyz: [f64; 4],
    pub sigma_step_trace: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct Route {
    pub frames: Vec<String>,
    pub steps: Vec<RouteStep>,
    pub iso_wxyz: [f64; 4],
    pub translation_m: [f64; 3],
    pub precision_cost: f64,
    pub version_score: i64,
    pub tie_breaker: String,
}

fn compose_chain(arcs: &[&RouteArc]) -> (Iso, Cov6) {
    let mut iso = Iso::identity();
    let mut cov = Cov6::zero();
    for a in arcs {
        iso = iso.compose(&a.iso);
        let next = compose_covariance(&iso, &cov, &a.iso, &a.cov);
        cov = next;
    }
    (iso, cov)
}

/// Covariance of a composed transform.  Because cross-correlations between
/// independently estimated edges are unknown, we propagate with the
/// first-order Jacobians and treat edges as uncorrelated (standard, and the
/// conservative choice for route scoring).
pub fn compose_covariance(
    total: &Iso,
    total_cov: &Cov6,
    next: &Iso,
    next_cov: &Cov6,
) -> Cov6 {
    use crate::math::{mat_mul, mat_t, rot_jac_wrt_angle, vcross};
    // Combined transform: y = R_t * (R_n x + t_n) + t_t
    // Params ordered [t_t(3), theta_t(3), t_n(3), theta_n(3)].
    let r_t = total.q.rotation_matrix();
    let r_n = next.q.rotation_matrix();
    let mut j = vec![vec![0.0f64; 12]; 6];
    // d t_out / d t_t = I ; d theta_out / d theta_t = I
    for i in 0..3 {
        j[i][i] = 1.0;
        j[i + 3][i + 3] = 1.0;
        // translation from next: R_t
        for k in 0..3 {
            j[i][6 + k] = r_t[i][k];
        }
        // rotation from next: R_t R_n reduced — use R_t on rotation vector
        // approximation (small increments compose additively in local frame;
        // after rotation R_t the increment is R_t * delta_theta_n).
        for k in 0..3 {
            j[i + 3][9 + k] = r_t[i][k];
        }
    }
    // d t_out / d theta_t = R_t * (-[R_n x + t_n]_x); evaluated at x=0
    let lever = next.t;
    let jac_t_angle = rot_jac_wrt_angle(&r_t, lever);
    for i in 0..3 {
        for k in 0..3 {
            j[i][3 + k] = jac_t_angle[i][k];
        }
    }
    // d t_out / d theta_n = R_t * R_n * (-[x]_x) at x=0 -> 0; keep for
    // completeness via lever inside next local frame (zero here).
    let _ = vcross;
    let rn_rt = crate::math::mat3_mul(&r_t, &r_n);
    let jac_n_angle = rot_jac_wrt_angle(&rn_rt, [0.0, 0.0, 0.0]);
    for i in 0..3 {
        for k in 0..3 {
            j[i][9 + k] += jac_n_angle[i][k];
        }
    }

    let mut big = vec![vec![0.0f64; 12]; 12];
    for i in 0..6 {
        for kk in 0..6 {
            big[i][kk] = total_cov.0[i][kk];
            big[6 + i][6 + kk] = next_cov.0[i][kk];
        }
    }
    let jt = mat_t(&j);
    let tmp = mat_mul(&j, &big);
    let out = mat_mul(&tmp, &jt);
    let mut c = [[0.0f64; 6]; 6];
    for i in 0..6 {
        for kk in 0..6 {
            c[i][kk] = out[i][kk];
        }
    }
    Cov6(c)
}

/// Enumerate all simple (frame-simple) paths `from -> to` over the given arcs,
/// deterministically, and return them ranked.
pub fn enumerate_routes(arcs: &[RouteArc], from: &str, to: &str) -> Vec<Route> {
    let mut adj: std::collections::BTreeMap<&str, Vec<&RouteArc>> =
        std::collections::BTreeMap::new();
    for a in arcs {
        adj.entry(a.source.as_str()).or_default().push(a);
    }
    for v in adj.values_mut() {
        // Deterministic neighbour order: target name, then version rank.
        v.sort_by(|a, b| {
            a.target
                .cmp(&b.target)
                .then(b.version_rank.cmp(&a.version_rank))
                .then(a.arc_id.cmp(&b.arc_id))
        });
        v.dedup_by(|a, b| a.target == b.target && a.arc_id == b.arc_id);
    }

    let mut found: Vec<Vec<&RouteArc>> = Vec::new();
    let mut stack: Vec<(&str, Vec<&RouteArc>)> = vec![(from, Vec::new())];
    let max_depth = 16;
    while let Some((node, path)) = stack.pop() {
        if node == to && !path.is_empty() {
            found.push(path);
            continue;
        }
        if path.len() >= max_depth {
            continue;
        }
        if let Some(nexts) = adj.get(node) {
            for a in nexts.iter().rev() {
                let visited = path.iter().any(|x| x.target == a.target)
                    || a.target == from;
                if visited {
                    continue;
                }
                let mut p = path.clone();
                p.push(a);
                stack.push((a.target.as_str(), p));
            }
        }
    }

    let mut routes: Vec<Route> = found
        .into_iter()
        .map(|path| {
            let refs: Vec<&RouteArc> = path.iter().copied().collect();
            let (iso, cov) = compose_chain(&refs);
            let mut frames = vec![from.to_string()];
            let mut steps = Vec::new();
            let mut version_score = 0;
            let mut acc = Iso::identity();
            let mut acc_cov = Cov6::zero();
            for a in &refs {
                acc = acc.compose(&a.iso);
                acc_cov = compose_covariance(&acc, &acc_cov, &a.iso, &a.cov);
                steps.push(RouteStep {
                    from: a.source.clone(),
                    to: a.target.clone(),
                    arc_id: a.arc_id.clone(),
                    translation_m: a.iso.t,
                    quat_wxyz: a.iso.q.0,
                    sigma_step_trace: a.cov.trace_cost(),
                });
                frames.push(a.target.clone());
                version_score += a.version_rank;
            }
            let tie = frames.join(">");
            Route {
                frames,
                steps,
                iso_wxyz: iso.q.0,
                translation_m: iso.t,
                precision_cost: cov.trace_cost(),
                version_score,
                tie_breaker: tie,
            }
        })
        .collect();

    // Selection rule (explicit, map-order independent):
    // 1. smallest propagated covariance trace (best precision);
    // 2. on tie, highest summed version rank (prefer newer calibrations);
    // 3. final tie-break on the lexicographic frame chain.
    routes.sort_by(|a, b| {
        a.precision_cost
            .partial_cmp(&b.precision_cost)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(b.version_score.cmp(&a.version_score))
            .then(a.tie_breaker.cmp(&b.tie_breaker))
    });
    routes
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quat;

    fn arc(s: &str, t: &str, cost: f64, rank: i64) -> RouteArc {
        RouteArc {
            source: s.into(),
            target: t.into(),
            iso: Iso::new(Quat::identity(), [0.0, 0.0, 0.0]),
            cov: Cov6::diagonal([cost; 3], [cost * 1e-4; 3]),
            arc_id: format!("{s}->{t}@{cost}"),
            version_rank: rank,
        }
    }

    #[test]
    fn picks_best_precision_and_keeps_candidates() {
        let arcs = vec![
            arc("a", "x", 1.0, 1),
            arc("x", "m", 1.0, 1),
            arc("a", "m", 5.0, 9),
        ];
        let r = enumerate_routes(&arcs, "a", "m");
        assert!(r.len() >= 2);
        // two cheap edges (trace 2e-...) beat the single costly edge
        assert_eq!(r[0].frames, vec!["a", "x", "m"]);
    }

    #[test]
    fn equal_precision_uses_version_rule() {
        let arcs = vec![
            arc("a", "x", 1.0, 5),
            arc("x", "m", 1.0, 5),
            arc("a", "y", 1.0, 1),
            arc("y", "m", 1.0, 1),
        ];
        let r = enumerate_routes(&arcs, "a", "m");
        assert_eq!(r[0].frames, vec!["a", "x", "m"]);
    }

    #[test]
    fn rejects_cycle_with_residual() {
        let e = |s: &str, t: &str, tx: f64| Edge {
            id: 0,
            source: s.into(),
            target: t.into(),
            version: 1,
            valid_from: None,
            valid_to: None,
            iso: Iso::new(Quat::identity(), [tx, 0.0, 0.0]),
            cov: Cov6::diagonal([1e-6; 3], [1e-8; 3]),
            origin: "test".into(),
            created_at: 0.0,
        };
        // m -> b -> a exists; adding a -> m closes a loop with 0.3 m residual
        let edges = vec![e("m", "b", 0.1), e("b", "a", 0.2)];
        let rep = find_cycle(&edges, "a", "m").unwrap();
        assert!(rep.frames.first().unwrap() == "a");
        assert!((rep.residual_translation_m - 0.3).abs() < 1e-9);
    }
}
