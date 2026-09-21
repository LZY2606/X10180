mod common;
use common::*;

#[test]
fn restart_reuses_sequence_numbers_in_new_generation() {
    let app = app();
    stream(&app, "lidar-A", "lidar", "lidar");
    lidar_packet(&app, 0, 10, vec![[0.0, 0.0, 0.0]], false, "a");
    lidar_packet(&app, 100, 11, vec![[0.0, 0.0, 0.0]], false, "b");
    lidar_packet(&app, 200, 0, vec![[0.0, 0.0, 0.0]], true, "reboot");
    let gens: Vec<(i64, String)> = {
        let c = app.db.0.lock().unwrap();
        let mut s = c.prepare("SELECT generation,flag FROM packets ORDER BY received_order").unwrap();
        s.query_map([], |r| Ok((r.get(0)?, r.get(1)?))).unwrap()
            .map(|r| r.unwrap()).collect()
    };
    assert_eq!(gens[0].0, 1);
    assert_eq!(gens[1].0, 1);
    assert_eq!(gens[2].0, 2);
    assert_eq!(gens[2].1, "generation_restart");
}

#[test]
fn duplicate_packet_is_stored_but_not_derived_twice() {
    let app = app();
    stream(&app, "lidar-A", "lidar", "lidar");
    // A genuine replay: identical timestamp, seq and content arrives again.
    let a = lidar_packet(&app, 100, 5, vec![[1.0, 0.0, 0.0]], true, "same");
    let b = lidar_packet(&app, 100, 5, vec![[1.0, 0.0, 0.0]], false, "same");
    assert_ne!(a, b);
    let c = app.db.0.lock().unwrap();
    let (np, nf): (i64, i64) = c
        .query_row("SELECT (SELECT COUNT(*) FROM packets), (SELECT COUNT(*) FROM frames)", [],
                   |r| Ok((r.get(0)?, r.get(1)?)))
        .unwrap();
    assert_eq!(np, 2, "raw duplicate retained");
    assert_eq!(nf, 1, "duplicate spawns no second frame");
}

#[test]
fn large_time_gap_splits_generation_without_restart_flag() {
    let app = app();
    stream(&app, "lidar-A", "lidar", "lidar");
    lidar_packet(&app, 0, 0, vec![[0.0; 3]], true, "g1");
    lidar_packet(&app, 5_000, 1, vec![[0.0; 3]], false, "g2 after gap");
    let gens: Vec<(i64, String)> = {
        let c = app.db.0.lock().unwrap();
        let mut s = c.prepare("SELECT generation,flag FROM packets ORDER BY received_order").unwrap();
        s.query_map([], |r| Ok((r.get(0)?, r.get(1)?))).unwrap()
            .map(|r| r.unwrap()).collect()
    };
    assert_eq!(gens[1].0, 2);
    assert_eq!(gens[1].1, "generation_gap");
}
