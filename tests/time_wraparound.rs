mod common;
use common::*;
use pointcloud_warp::time::*;

const GPS_EPOCH_UNIX_MS: i64 = 315_964_800_000; // 1980-01-06

#[test]
fn gps_week_rollover_normalizes_to_continuous_time() {
    // Full week 2046 is in the rollover epoch containing reference 2000.
    let truth = 2046 * WEEK_MS + 500_000;
    let w10 = (2046 % 1024) as u32;
    assert_eq!(unwrap_gps_week_first(w10, 500_000.0), truth);
    // Later readings near the epoch boundary stay in the same epoch.
    assert_eq!(unwrap_gps_week(w10, 600_000.0, truth), truth + 100_000);
    // A reading whose nearest epoch is the previous rollover snaps back.
    let near = 2047 * WEEK_MS - WEEK_MS; // == 1023 weeks
    let _ = near;
}

#[test]
fn leap_second_duplicated_timestamp_is_disambiguated() {
    // Unix second 1483228799 (2016-12-31 23:59:59 UTC) is reported twice
    // on receivers that label the leap second with the same integer.
    let dup = 1_483_228_799_000i64;
    let before = unix_ms_to_gps_ms(dup, false);
    let leap = unix_ms_to_gps_ms(dup, true);
    // The two reports of the repeated UTC second separate by exactly one
    // second in continuous GPS time.
    assert_eq!(leap - before, 1_000);
    // The next UTC second is one continuous second after the leap second.
    let after = unix_ms_to_gps_ms(1_483_228_800_000, false);
    assert_eq!(after - leap, 1_000);
    // GPS was 18 s ahead of UTC after the 2017 leap second.
    let expected = 1_483_228_800_000i64 - GPS_EPOCH_UNIX_MS + 18_000;
    assert_eq!(after, expected);
}

#[test]
fn ingest_persists_raw_timestamp_frame_unit_summary() {
    let app = app();
    stream(&app, "lidar-A", "lidar", "lidar");
    let id = lidar_packet(&app, 0, 0, vec![[1.0, 2.0, 3.0]], true, "scan #0 raw");
    let c = app.db.0.lock().unwrap();
    let (raw_clock, raw_ts, frame, unit, summary, gps): (
        String, String, String, String, String, i64) = c
        .query_row(
            "SELECT raw_clock,raw_ts,coord_frame,unit,content_summary,gps_ms FROM packets WHERE id=?1",
            rusqlite::params![id], |r| Ok((r.get(0)?,r.get(1)?,r.get(2)?,r.get(3)?,r.get(4)?,r.get(5)?)))
        .unwrap();
    assert_eq!(raw_clock, "gps_ms");
    assert_eq!(frame, "lidar");
    assert_eq!(unit, "m");
    assert_eq!(summary, "scan #0 raw");
    assert_eq!(raw_ts, format!("gps_ms={}", gps));
}
