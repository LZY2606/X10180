//! 点云经纬 核心库（HTTP 层见 `web`，可执行入口见 `main`）。

pub mod db;
pub mod demo;
pub mod derive;
pub mod geo;
pub mod graph;
pub mod ingest;
pub mod query;
pub mod time;
pub mod web;

pub const INDEX_HTML: &str = include_str!("../static/index.html");
