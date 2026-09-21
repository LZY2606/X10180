use pointcloud_warp::{db, demo, service, web};

use std::net::SocketAddr;

use service::App;
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut addr: SocketAddr = "127.0.0.1:5240".parse().unwrap();
    let mut db_path = "pointcloud-warp.db".to_string();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--addr" => addr = args.next().expect("missing addr value").parse()?,
            "--db" => db_path = args.next().expect("missing db value"),
            "--help" | "-h" => {
                println!("pointcloud-warp [--addr 127.0.0.1:5240] [--db path.db]");
                return Ok(());
            }
            other => return Err(format!("unknown argument {}", other).into()),
        }
    }

    let db = Arc::new(db::open(&db_path)?);
    let app = Arc::new(App::new(db));
    // First run seeds a built-in demo dataset; on an existing database it
    // only recovers interrupted blocks.
    demo::seed(&app);

    let listener = tokio::net::TcpListener::bind(addr).await?;
    println!("点云经纬 listening on http://{} (db {})", addr, db_path);
    axum::serve(listener, web::router(app)).await?;
    Ok(())
}
