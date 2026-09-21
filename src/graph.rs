//! Transform graph: frame nodes, versioned rigid edges with validity
//! windows, cycle rejection with accumulated residuals, deterministic
//! best-path selection by explicit precision and version rules, and pose
//! interpolation that never crosses collection generations or exceeds a
//! configured maximum interval.

use crate::math::{self, Mat6, Rigid};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Edge {
    pub id: i64,
    pub name: String,
    pub from_frame: String,
    pub to_frame: String,
    pub version: i64,
    pub supersedes: Option<i64>,
    pub valid_start: Option<i64>,
    pub valid_end: Option<i64>,
    pub dynamic: bool,
    pub pose_stream: Option<i64>,
    pub transform: Rigid,
    pub covariance: Mat6,
    pub created_order: i64,
}

impl Edge {
    pub fn valid_at(&self, t: i64) -> bool {
        self.valid_start.map_or(true, |s| t >= s)
            && self.valid_end.map_or(true, |e| t < e)
    }

    /// Half-open interval overlap against [start, end). `None` bounds are
    /// open ranges.
    pub fn overlaps(&self, start: Option<i64>, end: Option<i64>) -> bool {
        let lo = match (self.valid_start, start) {
            (Some(a), Some(b)) => a.max(b),
            (Some(a), None) => a,
            (None, Some(b)) => b,
            (None, None) => return true,
        };
        let hi = match (self.valid_end, end) {
            (Some(a), Some(b)) => a.min(b),
            _ => return true,
        };
        lo < hi
    }
}

#[derive(Clone, Debug)]
pub struct PoseSample {
    pub time_ms: i64,
    pub generation: i64,
    pub transform: Rigid,
}

/// One traversed edge within a found path.
#[derive(Clone, Debug, Serialize)]
pub struct PathHop {
    pub edge_id: i64,
    pub name: String,
    pub version: i64,
    pub from_frame: String,
    pub to_frame: String,
    pub reversed: bool,
    pub interpolated: bool,
    pub time_ms: Option<i64>,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub qw: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct FoundPath {
    pub frames: Vec<String>,
    pub hops: Vec<PathHop>,
    pub transform: TransformDto,
    pub covariance: Vec<f64>,
    pub precision_trace: f64,
    pub version_key: Vec<i64>,
    pub selected: bool,
    pub rank_reason: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct TransformDto {
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub qw: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

impl From<&Rigid> for TransformDto {
    fn from(r: &Rigid) -> Self {
        TransformDto {
            tx: r.t[0],
            ty: r.t[1],
            tz: r.t[2],
            qw: r.q[0],
            qx: r.q[1],
            qy: r.q[2],
            qz: r.q[3],
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct CycleReport {
    pub frames: Vec<String>,
    pub edge_ids: Vec<i64>,
    pub residual: TransformDto,
    pub residual_translation_norm: f64,
    pub residual_rotation_rad: f64,
}

/// Detect whether adding `candidate` closes a loop among *static* edges.
/// Only static edges participate: dynamic pose streams legitimately
/// provide parallel observation paths and are not cycles.
///
/// We search existing edges (both traversal directions allowed, validity
/// windows must overlap the candidate window) for a simple path from the
/// candidate's `to_frame` back to its `from_frame`. Composing that path
/// `T_back` (to_frame -> from_frame) with the candidate gives the loop
/// closure `T_loop = T_back ∘ candidate`, whose deviation from identity
/// is the accumulated loop residual.
pub fn detect_cycle(static_edges: &[Edge], candidate: &Edge) -> Option<CycleReport> {
    let window = (candidate.valid_start, candidate.valid_end);

    fn step<'a>(
        node: &str,
        goal: &str,
        acc: Rigid,
        frames: &mut Vec<String>,
        ids: &mut Vec<i64>,
        edges: &[&'a Edge],
        candidate_id: i64,
        window: (Option<i64>, Option<i64>),
    ) -> Option<(Vec<String>, Vec<i64>, Rigid)> {
        if node == goal {
            return Some((frames.clone(), ids.clone(), acc));
        }
        for e in edges {
            if e.id == candidate_id || ids.contains(&e.id) || !e.overlaps(window.0, window.1) {
                continue;
            }
            if e.from_frame == node {
                frames.push(e.to_frame.clone());
                ids.push(e.id);
                if let Some(r) = step(&e.to_frame, goal, acc.compose(&e.transform), frames, ids,
                                      edges, candidate_id, window) {
                    return Some(r);
                }
                ids.pop();
                frames.pop();
            } else if e.to_frame == node {
                frames.push(e.from_frame.clone());
                ids.push(e.id);
                if let Some(r) = step(&e.from_frame, goal, e.transform.inverse().compose(&acc),
                                      frames, ids, edges, candidate_id, window) {
                    return Some(r);
                }
                ids.pop();
                frames.pop();
            }
        }
        None
    }

    let frames = vec![candidate.from_frame.clone(), candidate.to_frame.clone()];
    let mut ids: Vec<i64> = Vec::new();
    // acc maps current node to... track transform from candidate.to_frame
    // to the current node. Start identity at to_frame.
    let found = step(&candidate.to_frame, &candidate.from_frame, Rigid::identity(),
                     &mut vec![candidate.to_frame.clone()], &mut ids, &static_edges.iter().collect::<Vec<_>>(),
                     candidate.id, window);
    let _ = frames;

    found.map(|(fr, ids, back_xf)| {
        // back_xf: to_frame -> from_frame. Loop closure in from_frame:
        // candidate maps from->to, back maps to->from.
        let loop_xf = back_xf.compose(&candidate.transform);
        let angle = 2.0 * loop_xf.q[0].clamp(-1.0, 1.0).acos();
        CycleReport {
            frames: fr,
            edge_ids: ids,
            residual_translation_norm: math::vdot(loop_xf.t, loop_xf.t).sqrt(),
            residual_rotation_rad: angle,
            residual: TransformDto::from(&loop_xf),
        }
    })
}

/// Result of pose interpolation.
#[derive(Debug)]
pub struct InterpPose {
    pub transform: Rigid,
    pub prev_id: i64,
    pub next_id: i64,
    pub prev_time: i64,
    pub next_time: i64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum InterpError {
    NoSamples,
    CrossesGeneration { left_gen: i64, right_gen: i64 },
    IntervalTooLarge { gap_ms: i64, max_ms: i64 },
    OutOfRange { time_ms: i64, first: i64, last: i64 },
}

/// Interpolate a body->map pose at `time_ms`. Never extrapolates, never
/// crosses a generation boundary, never spans a gap larger than
/// `max_interval_ms`. Samples must be sorted by time.
pub fn interpolate_pose(
    samples: &[PoseSample],
    time_ms: i64,
    max_interval_ms: i64,
) -> Result<InterpPose, InterpError> {
    if samples.is_empty() {
        return Err(InterpError::NoSamples);
    }
    let first = samples.first().unwrap().time_ms;
    let last = samples.last().unwrap().time_ms;
    if time_ms < first || time_ms > last {
        return Err(InterpError::OutOfRange { time_ms, first, last });
    }
    let pos = samples.binary_search_by(|s| s.time_ms.cmp(&time_ms));
    let idx = match pos {
        Ok(i) => i,
        Err(i) => i - 1,
    };
    if idx + 1 >= samples.len() && pos.is_err() {
        return Err(InterpError::OutOfRange { time_ms, first, last });
    }
    let left = &samples[idx];
    let right = if pos.is_ok() {
        &samples[idx]
    } else {
        &samples[idx + 1]
    };
    if pos.is_ok() {
        return Ok(InterpPose {
            transform: left.transform.clone(),
            prev_id: idx as i64,
            next_id: idx as i64,
            prev_time: left.time_ms,
            next_time: left.time_ms,
        });
    }
    if left.generation != right.generation {
        return Err(InterpError::CrossesGeneration {
            left_gen: left.generation,
            right_gen: right.generation,
        });
    }
    let gap = right.time_ms - left.time_ms;
    if gap > max_interval_ms {
        return Err(InterpError::IntervalTooLarge { gap_ms: gap, max_ms: max_interval_ms });
    }
    let f = (time_ms - left.time_ms) as f64 / gap as f64;
    let xf = Rigid {
        t: [
            left.transform.t[0] + (right.transform.t[0] - left.transform.t[0]) * f,
            left.transform.t[1] + (right.transform.t[1] - left.transform.t[1]) * f,
            left.transform.t[2] + (right.transform.t[2] - left.transform.t[2]) * f,
        ],
        q: math::slerp(left.transform.q, right.transform.q, f),
    };
    Ok(InterpPose {
        transform: xf,
        prev_id: idx as i64,
        next_id: (idx + 1) as i64,
        prev_time: left.time_ms,
        next_time: right.time_ms,
    })
}

/// Find every simple path from `source` to `target` valid at `time_ms`,
/// rank them by explicit precision then deterministic version/name rules,
/// and mark exactly one path selected. Dynamic edges are expanded with
/// interpolated poses; the same static frame graph may legitimately offer
/// parallel dynamic paths and all of them are retained as candidates.
///
/// `static_edges` is required to be acyclic (enforced on insertion); to
/// keep traversal bounded and deterministic we still cap simple paths at
/// `max_hops` and sort every adjacency list.
pub fn find_paths(
    static_edges: &[Edge],
    dynamic_edges: &[Edge],
    poses_by_stream: &std::collections::HashMap<i64, Vec<PoseSample>>,
    source: &str,
    target: &str,
    time_ms: i64,
    max_interval_ms: i64,
    max_hops: usize,
) -> Result<Vec<FoundPath>, String> {
    if source == target {
        return Ok(vec![FoundPath {
            frames: vec![source.to_string()],
            hops: vec![],
            transform: TransformDto::from(&Rigid::identity()),
            covariance: Mat6::identity().to_flat(),
            precision_trace: 0.0,
            version_key: vec![],
            selected: true,
            rank_reason: "identity".into(),
        }]);
    }

    // Build adjacency: entries are (edge, reversed, resolved transform).
    struct Adj {
        from: String,
        to: String,
        edge_id: i64,
        name: String,
        version: i64,
        reversed: bool,
        xf: Rigid,
        cov: Mat6,
        interpolated: bool,
        interp_time: Option<i64>,
    }
    let mut adj: Vec<Adj> = Vec::new();
    for e in static_edges {
        if !e.valid_at(time_ms) {
            continue;
        }
        adj.push(Adj {
            from: e.from_frame.clone(), to: e.to_frame.clone(), edge_id: e.id,
            name: e.name.clone(), version: e.version,
            reversed: false, xf: e.transform.clone(), cov: e.covariance.clone(),
            interpolated: false, interp_time: None,
        });
        adj.push(Adj {
            from: e.to_frame.clone(), to: e.from_frame.clone(), edge_id: e.id,
            name: e.name.clone(), version: e.version,
            reversed: true, xf: e.transform.inverse(),
            cov: e.covariance.clone(),
            interpolated: false, interp_time: None,
        });
    }
    for e in dynamic_edges {
        if !e.valid_at(time_ms) {
            continue;
        }
        let stream = match e.pose_stream {
            Some(s) => s,
            None => continue,
        };
        let samples = match poses_by_stream.get(&stream) {
            Some(s) => s,
            None => continue,
        };
        let interp = match interpolate_pose(samples, time_ms, max_interval_ms) {
            Ok(v) => v,
            Err(_) => continue,
        };
        let base = if e.dynamic { &interp.transform } else { &e.transform };
        // Body->map pose defines child(body)=from_frame to map=to_frame.
        adj.push(Adj {
            from: e.from_frame.clone(), to: e.to_frame.clone(), edge_id: e.id,
            name: e.name.clone(), version: e.version,
            reversed: false, xf: base.clone(), cov: e.covariance.clone(),
            interpolated: true, interp_time: Some(time_ms),
        });
        adj.push(Adj {
            from: e.to_frame.clone(), to: e.from_frame.clone(), edge_id: e.id,
            name: e.name.clone(), version: e.version,
            reversed: true, xf: base.inverse(), cov: e.covariance.clone(),
            interpolated: true, interp_time: Some(time_ms),
        });
    }

    // Deterministic adjacency ordering: node name, then (edge_id, reversed).
    adj.sort_by(|a, b| {
        a.from.cmp(&b.from)
            .then(a.to.cmp(&b.to))
            .then(a.edge_id.cmp(&b.edge_id))
            .then(a.reversed.cmp(&b.reversed))
    });

    let mut complete: Vec<(Vec<usize>, Rigid, Mat6)> = Vec::new();

    fn walk(
        node: &str,
        target: &str,
        path: &mut Vec<usize>,
        used_edges: &mut Vec<i64>,
        xf: Rigid,
        cov: Mat6,
        adj: &[&Adj],
        max_hops: usize,
        complete: &mut Vec<(Vec<usize>, Rigid, Mat6)>,
    ) {
        if node == target {
            complete.push((path.clone(), xf, cov));
            return;
        }
        if path.len() >= max_hops {
            return;
        }
        for (idx, a) in adj.iter().enumerate() {
            if a.from != node || used_edges.contains(&a.edge_id) {
                continue;
            }
            path.push(idx);
            used_edges.push(a.edge_id);
            let new_cov = math::compose_cov(&xf, &cov, &a.cov);
            walk(&a.to, target, path, used_edges, xf.compose(&a.xf), new_cov,
                 adj, max_hops, complete);
            used_edges.pop();
            path.pop();
        }
    }

    let refs: Vec<&Adj> = adj.iter().collect();
    walk(source, target, &mut Vec::new(), &mut Vec::new(), Rigid::identity(),
         Mat6::zero(), &refs, max_hops, &mut complete);

    let mut paths: Vec<FoundPath> = complete
        .into_iter()
        .map(|(idxs, xf, cov)| {
            let mut frames = vec![source.to_string()];
            let mut hops = Vec::new();
            let mut versions = Vec::new();
            for i in idxs {
                let a = &adj[i];
                frames.push(a.to.clone());
                versions.push(a.version);
                let t = if i == 0 { &a.xf } else { &a.xf };
                hops.push(PathHop {
                    edge_id: a.edge_id,
                    name: a.name.clone(),
                    version: a.version,
                    from_frame: a.from.clone(),
                    to_frame: a.to.clone(),
                    reversed: a.reversed,
                    interpolated: a.interpolated,
                    time_ms: a.interp_time,
                    tx: t.t[0],
                    ty: t.t[1],
                    tz: t.t[2],
                    qw: t.q[0],
                    qx: t.q[1],
                    qy: t.q[2],
                    qz: t.q[3],
                });
            }
            FoundPath {
                frames,
                hops,
                transform: TransformDto::from(&xf),
                covariance: cov.to_flat(),
                precision_trace: cov.trace(),
                version_key: versions,
                selected: false,
                rank_reason: String::new(),
            }
        })
        .collect();

    if paths.is_empty() {
        return Err(format!("no transform path from {} to {}", source, target));
    }

    // Deterministic ranking: lower propagated uncertainty (explicit
    // precision) wins; ties resolved by newer edge versions first
    // (lexicographic on the version sequence), then by frame/edge names.
    paths.sort_by(|a, b| {
        a.precision_trace
            .partial_cmp(&b.precision_trace).unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| b.version_key.cmp(&a.version_key))
            .then_with(|| a.frames.cmp(&b.frames))
            .then_with(|| {
                let ak: Vec<&str> = a.hops.iter().map(|h| h.name.as_str()).collect();
                let bk: Vec<&str> = b.hops.iter().map(|h| h.name.as_str()).collect();
                ak.cmp(&bk)
            })
    });
    paths[0].selected = true;
    paths[0].rank_reason = format!(
        "lowest propagated trace {:.6e}; tie-break by newest versions then fixed name order",
        paths[0].precision_trace
    );
    for p in paths.iter_mut().skip(1) {
        p.rank_reason = "candidate: higher propagated uncertainty or deterministic tie-break".into();
    }
    Ok(paths)
}
