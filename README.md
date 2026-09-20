# 点云经纬（dianyun-jingwei）

按包切分的 LiDAR 点、GNSS 姿态与设备标定的**可核验帧链**对齐系统。
Rust + Axum + SQLite，单二进制提供 API 与内嵌前端，运行时不下载任何地图或点云。

## 运行

```bash
cargo build --all-targets
cargo test --all-targets
cargo run -- --addr 127.0.0.1:5240
# 浏览器访问 http://127.0.0.1:5240 （页面标题“点云经纬”）
```

首次启动（空库）自动播种演示数据，覆盖：GPS 10 位周翻转、闰秒标签、
设备重启后序号归零、迟到包、重叠包、姿态时间缺口、毫米单位点云。

可选参数：`--db <path.db>`（默认 `dianyun-jingwei.db`）、`--no-seed`。

## 设计要点

- **原始事实与派生缓存分离**：`packet/raw_point/pose_sample/edge` 只存导入与标定事实；
  `block/derived_point/point_chain_step` 是派生缓存，按块原子提交，
  中断后遗留的 `building` 半块在下次构建时整体删除重来，不会被当成成功。
- **先分代次再排包**：序号重置/时间倒流开启采集代次；迟到包归回旧代次、
  重叠包标记 `is_duplicate`；姿态插值不跨姿态代次、间隔不得超过 0.2 s。
- **时间环绕**：GPS 周按设备上一包展开 10/13 位翻转；UTC 闰秒（23:59:60.x）
  映射到严格全序的连续刻度。
- **变换图**：边带有效时间窗与 6×6 协方差；加边做结构性环路检测，
  成环时返回帧序列、平移/旋转残差与路径协方差迹并拒绝写入。
- **确定性选路**：用 `BTreeMap/BTreeSet` 枚举全部简单路径，按
  “协方差迹 → 版本号之和 → 帧名字典序”排序，未选路径保留为候选，
  不依赖 map 遍历顺序。
- **可溯源**：每个派生点记录原始坐标与单位、map 坐标、位置协方差、
  经过的每条边（key/版本/有效窗/协方差迹）；点击点即可查看来源链。
  误差传播采用 SE(3) 伴随 `Σ_g = AdᵀΣ_a Ad + Σ_b`，点位置 `Σ_p = J Σ Jᵀ`。
- **局部失效**：标定修订生成新版本，仅使时间窗重叠且链上用到该 key 的
  ready 块失效，其余块复用。

## HTTP API

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/stats` `/api/generations` `/api/packets?kind=` | 概览与原始包 |
| GET | `/api/gaps?threshold=` | 同代次内时间缺口 |
| GET | `/api/points?limit=` `/api/point/:id` | 帧着色点云与点来源链 |
| GET | `/api/trajectory` `/api/edges?all=` `/api/events` | 轨迹、标定版本、事件 |
| GET | `/api/path?source=&target=&at=` | 路径选择与候选 |
| POST | `/api/ingest` | 导入一个数据包（保留原始时间/坐标系/单位） |
| POST | `/api/edge` | 新增标定/地图边（自动升版本；成环返回 409） |
| POST | `/api/rebuild` | 恢复半块并重建缺失派生块 |

导入包示例：

```json
{
  "device_id": "DEV-01",
  "kind": "lidar",
  "seq": 12,
  "time": { "kind": "gps_week_tow", "week": 2345, "tow": 123456.7, "week_bits": 10 },
  "coord_system": "sensor",
  "unit": "m",
  "points": [[1.0, 2.0, 3.0, 0.8]]
}
```

## 测试

`tests/end_to_end.rs` 覆盖时间环绕、重复序号、迟到/重叠、路径并列、奇异协方差、
单位错误、跨代次/超间隔插值、局部失效与崩溃恢复；几何断言使用显式容差（1e-6 m）。
