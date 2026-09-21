mod common;
use common::*;

fn edge(name: &str, from: &str, to: &str, tx: f64) -> EdgeInput {
    EdgeInput {
        name: name.into(), from_frame: from.into(), to_frame: to.into(),
        valid_start: None, valid_end: None,
        tx, ty: 0.0, tz: 0.0, qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
        covariance: cov(1e-4),
    }
}

#[test]
fn closing_edge_is_rejected_with_loop_and_residual() {
    let app = app();
    // a -> b -> c -> a would close; consistent a->b (1.0), b->c (1.0),
    // and an imperfect c->a (-1.99) leaves a measurable residual.
    app.add_edge(&edge("ab", "a", "b", 1.0)).unwrap();
    app.add_edge(&edge("bc", "b", "c", 1.0)).unwrap();
    let err = app.add_edge(&edge("ca", "c", "a", -1.99)).unwrap_err();
    assert!(err.error.contains("closes a frame loop"));
    // Existing path a->b->c sums to 2.0; candidate c->a is -1.99, so the
    // loop translation residual is ~0.01.
    assert!(err.cycle.residual_translation_norm > 1e-4,
        "residual {}", err.cycle.residual_translation_norm);
    assert!(err.cycle.residual_translation_norm < 0.1);
    assert!(err.cycle.edge_ids.len() >= 2, "loop reports its edges");
    assert!(err.cycle.frames.contains(&"a".to_string()));
}

#[test]
fn consistent_loop_has_near_zero_residual() {
    let app = app();
    app.add_edge(&edge("ab", "a", "b", 1.0)).unwrap();
    app.add_edge(&edge("bc", "b", "c", 1.0)).unwrap();
    let err = app.add_edge(&edge("ca", "c", "a", -2.0)).unwrap_err();
    assert!(err.cycle.residual_translation_norm < 1e-9,
        "perfect closure residual {}", err.cycle.residual_translation_norm);
}

#[test]
fn non_closing_edge_is_accepted() {
    let app = app();
    app.add_edge(&edge("ab", "a", "b", 1.0)).unwrap();
    app.add_edge(&edge("bc", "b", "c", 1.0)).unwrap();
    // d attaches as a new leaf: no loop.
    assert!(app.add_edge(&edge("cd", "c", "d", 0.5)).is_ok());
}

#[test]
fn dynamic_pose_parallel_paths_are_not_static_cycles() {
    let app = app();
    rig(&app);
    // Two dynamic body->map edges coexist (already added by rig). Adding
    // a second static calibration leaf is fine; only static loops error.
    assert!(app.add_edge(&edge("aux", "lidar", "aux", 0.1)).is_ok());
}
