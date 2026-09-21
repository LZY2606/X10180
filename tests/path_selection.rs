mod common;
use common::*;
use pointcloud_warp::math::Mat6;

fn build(app: &std::sync::Arc<App>, dt: i64) {
    pose_packet(app, "gnss-primary", dt - 100, 0, true, 1.0, 0.0, 0.0);
    pose_packet(app, "gnss-primary", dt + 100, 1, false, 1.0, 0.0, 0.0);
    pose_packet(app, "gnss-backup", dt - 100, 0, true, 1.0, 0.0, 0.0);
    pose_packet(app, "gnss-backup", dt + 100, 1, false, 1.0, 0.0, 0.0);
    lidar_packet(app, dt, 0, pts(3), true, "frame");
}

#[test]
fn precise_path_selected_backup_retained_as_candidate() {
    let app = app();
    rig(&app);
    build(&app, 200);
    let report = app.recompute();
    assert_eq!(report.built, 1);

    let paths = app.paths_at("lidar", "map", T0 + 200).unwrap();
    assert_eq!(paths.len(), 2, "both feasible paths retained");
    let chosen: Vec<_> = paths.iter().filter(|p| p.selected).collect();
    assert_eq!(chosen.len(), 1);
    let names: Vec<&str> = chosen[0].hops.iter().map(|h| h.name.as_str()).collect();
    assert!(names.contains(&"pose_primary"), "primary chosen: {:?}", names);
    assert!(paths[0].precision_trace < paths[1].precision_trace);
}

#[test]
fn exact_tie_is_resolved_deterministically_not_by_map_order() {
    let app = app();
    rig(&app);
    // Force identical covariances on both pose edges.
    {
        let c = app.db.0.lock().unwrap();
        c.execute("UPDATE edges SET cov_json=?1 WHERE dynamic=1",
                  rusqlite::params![serde_json::to_string(&common::parse(5e-4)).unwrap()])
            .unwrap();
    }
    build(&app, 200);
    let p1 = app.paths_at("lidar", "map", T0 + 200).unwrap();
    let p2 = app.paths_at("lidar", "map", T0 + 200).unwrap();
    assert_eq!(p1.len(), p2.len());
    for (a, b) in p1.iter().zip(p2.iter()) {
        assert_eq!(a.selected, b.selected);
        assert_eq!(a.frames, b.frames);
    }
    // Exactly one selected despite an exact precision tie.
    assert_eq!(p1.iter().filter(|p| p.selected).count(), 1);
    let chosen_name = &p1.iter().find(|p| p.selected).unwrap().hops[1].name;
    // Deterministic name tie-break: "pose_backup" < "pose_primary".
    assert_eq!(chosen_name, "pose_backup");
}

#[test]
fn covariance_trace_drives_ranking_with_rotation_growth() {
    let app = app();
    rig(&app);
    let mut big = Mat6::zero();
    for i in 0..6 { big.a[i][i] = 1e-2; }
    {
        let c = app.db.0.lock().unwrap();
        c.execute("UPDATE edges SET cov_json=?1 WHERE name='pose_primary'",
                  rusqlite::params![serde_json::to_string(&big).unwrap()])
            .unwrap();
    }
    build(&app, 200);
    let paths = app.paths_at("lidar", "map", T0 + 200).unwrap();
    let chosen = paths.iter().find(|p| p.selected).unwrap();
    assert_eq!(chosen.hops[1].name, "pose_backup",
        "backup now more precise, must win");
}
