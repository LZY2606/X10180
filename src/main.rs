//! 点云经纬 可执行入口：参数解析、建库、空库播种、启动 HTTP 服务。

use dianyun_jingwei::{db, demo, web};

#[derive(Debug)]
struct Args {
    addr: String,
    db_path: String,
    no_seed: bool,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            addr: "127.0.0.1:5240".into(),
            db_path: "dianyun-jingwei.db".into(),
            no_seed: false,
        }
    }
}

fn parse_args(argv: &[String]) -> Result<Args, String> {
    let mut a = Args::default();
    let mut i = 1;
    while i < argv.len() {
        match argv[i].as_str() {
            "--addr" => {
                i += 1;
                a.addr = argv.get(i).ok_or("--addr 需要地址参数")?.clone();
            }
            "--db" => {
                i += 1;
                a.db_path = argv.get(i).ok_or("--db 需要路径参数")?.clone();
            }
            "--no-seed" => a.no_seed = true,
            "--help" | "-h" => {
                return Ok(Args {
                    addr: String::new(),
                    ..Default::default()
                })
            }
            other => return Err(format!("未知参数 {other}（支持 --addr、--db、--no-seed）")),
        }
        i += 1;
    }
    Ok(a)
}

fn main() -> anyhow::Result<()> {
    let argv: Vec<String> = std::env::args().collect();
    let args = parse_args(&argv).map_err(|e| anyhow::anyhow!(e))?;
    if args.addr.is_empty() {
        println!(
            "点云经纬\n用法: {} [--addr 127.0.0.1:5240] [--db path.db] [--no-seed]",
            argv.first().map(String::as_str).unwrap_or("dianyun-jingwei")
        );
        return Ok(());
    }

    let database = db::open(&args.db_path)?;
    if !args.no_seed {
        match demo::seed_if_empty(&database) {
            Ok(true) => println!("空库：已播种演示数据包与默认标定"),
            Ok(false) => {}
            Err(e) => eprintln!("演示数据播种失败（继续启动空服务）: {e}"),
        }
    }

    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(web::serve(database, &args.addr))
}
