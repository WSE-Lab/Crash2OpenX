# 42 例库存中的 10 例 ADS 测试交付

交付入口：`output/delivery/ads_showcase_10_20260919/START_HERE.html`。同目录旁保存 ZIP、SHA256 与独立校验结果。来源为 `outputs/framework_42_20260917`，每例只计一个最终参数配置；未覆盖的库存与失败尝试不计入成果。

选中编号：013、035、038、055、097、120、135、144、262、293。每例包含 XODR、相对引用地图的 XOSC、原始 PDF/种子、派生参数、真实 CARLA/InterFuser 视频、轨迹、日志及运行代码快照。可运行测试需要 CARLA 0.9.16 和项目支持的 ScenarioRunner/PCLA 环境，完整命令见交付包 REPRODUCE.md。

本轮实现：

- 使用真实车身净空触发同向切入/减速；调整接近距离时保持附近骑行者的前后顺序。
- 根据生成车道路由中的冲突位置设置路口出发窗口；修复 oncoming 对向入口和同向前车转弯入口选择。
- 在动作启动之前安装确定的生成车道路由；弯道跟随连续道路采样。
- Act 任一停止条件满足即可结束，条件组内部保持 AND；只更新本任务的远端隔离运行环境。
- 可选 NPC 制动上限保持默认行为兼容。262 完全停车配置在低速处仍有非物理尖峰，已排除，最终采用 3.8 → 2.4 m/s 的减速变体，实测峰值减速度约 3.71 m/s²。
- 筛选同时检查输入哈希、原生几何、真实动作、NPC 道路走廊、入口方向、交互窗口及合理的前车减速度；ADS 碰撞/偏离如实记录为被测结果。

生成与检查工具：`tools/expand_ads_batch.py`、`tools/review_ads_expansion.py`、`tools/build_ads_review_assets.py`、`tools/package_ads_showcase.py`、`tools/validate_ads_showcase.py`。最终质量记录在 `outputs/ads_expansion_20260919/visual_reviews.json`，每个接受项绑定实际检查过的图、视频、XOSC、XODR 与质量报告的 SHA256。

验证：`uv run --with 'py-trees==0.8.3' python -m pytest -q`，283 passed，8 subtests passed。独立交付检查包括十对 XML schema、相对地图引用、原始上传输入一致性、全部文件与运行时代码哈希、视频完整解码、HTML 链接、ZIP 内容和 CRC。

边界：单次事故启发的测试变体，不是完整事故重建或统计失效率；038 仅交互窗口有效，完整运行在早期停止条件版本下达到墙钟超时；038/262 的 ADS 分别映射到报告中的卡车/后车。信号合规、精确碰撞部位和完整事故链尚未验收。画面来自实际仿真；车道图为生成 XODR 几何与实测轨迹的示意。
