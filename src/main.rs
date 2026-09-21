//! 点云经纬 — binary entry point.

use pointcloud_meridian::{db::Db, demo, server, service};
use std::{io::Write as _, sync::Arc};

struct Args {
    addr: String,
    db_path: String,
}

fn parse_args() -> Args {
    let mut addr = "127.0.0.1:5240".to_string();
    let mut db_path = "pointcloud-meridian.sqlite".to_string();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--addr" => {
                addr = it.next().unwrap_or_else(|| {
                    eprintln!("--addr requires a value");
                    std::process::exit(2);
                });
            }
            "--db" => {
                db_path = it.next().unwrap_or_else(|| {
                    eprintln!("--db requires a value");
                    std::process::exit(2);
                });
            }
            "-h" | "--help" => {
                println!(
                    "点云经纬 (pointcloud-meridian)\n\n\
                     USAGE:\n    pointcloud-meridian [--addr 127.0.0.1:5240] \
                     [--db PATH]\n\n\
                     Frontend and demo data are served from this process."
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    Args { addr, db_path }
}

#[tokio::main]
async fn main() {
    let args = parse_args();
    let db = Arc::new(Db::open(&args.db_path).unwrap_or_else(|e| {
        eprintln!("failed to open database {}: {e}", args.db_path);
        std::process::exit(1);
    }));

    // Crash recovery before anything touches derived blocks.
    {
        let c = db.lock();
        service::recover(&c).expect("recover");
    }
    // First run: seed the built-in, fully offline dataset.
    demo::seed(&db);

    let app = server::router(db.clone());
    let listener = tokio::net::TcpListener::bind(&args.addr)
        .await
        .unwrap_or_else(|e| {
            eprintln!("failed to bind {}: {e}", args.addr);
            std::process::exit(1);
        });
    let mut stdout = std::io::stdout().lock();
    let _ = writeln!(
        stdout,
        "点云经纬 listening on http://{} (db: {})",
        args.addr, args.db_path
    );
    axum::serve(listener, app).await.expect("server");
}
