use pointcloud_jingwei::server;
use pointcloud_jingwei::store::Store;
use std::sync::{Arc, Mutex};

#[tokio::main]
async fn main() {
    let mut addr = "127.0.0.1:5240".to_string();
    let mut db = "pointcloud.db".to_string();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--addr" if i + 1 < args.len() => {
                addr = args[i + 1].clone();
                i += 2;
            }
            "--db" if i + 1 < args.len() => {
                db = args[i + 1].clone();
                i += 2;
            }
            _ => i += 1,
        }
    }
    let store = Store::open(&db).expect("打开数据库失败");
    let app = server::app(Arc::new(Mutex::new(store)));
    let listener = tokio::net::TcpListener::bind(&addr).await.expect("绑定地址失败");
    println!("点云经纬 服务已启动: http://{addr}");
    axum::serve(listener, app).await.unwrap();
}
