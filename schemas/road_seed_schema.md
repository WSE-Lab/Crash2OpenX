# Road Seed Schema (simple) — **v2**

当前道路推理脚本 `tools/api_infer_road_seed.py` 实际使用的 road seed 定义。
只描述**道路几何**，不含车辆/行人/轨迹/坐标/OpenDRIVE 底层 ID（那些由下游 compiler 生成）。

> **v2 变更**：`lanes` 从单个 int（隐含双向对称）升级为方向感知对象 `{forward, backward}`，
> 并新增 `center_line`（中心线虚实）。动机：语料逼出大量单行道（one-way street）与"障碍物绕行"场景——
> 旧 schema 表达不了单向/不对称/可否借对向道，导致编译出无操作空间的道路。版本冻结为 v2。

## 结构

```json
{
  "topology": "cross_intersection",
  "type": "town",
  "lanes": { "forward": 1, "backward": 1 },
  "center_line": "broken"
}
```

`additionalProperties: false`，只允许这四个字段。

## 字段

| 字段 | 类型 | 必填 | 取值 | 默认 | 含义 |
| --- | --- | --- | --- | --- | --- |
| `topology` | enum | 是 | straight, curve, cross_intersection, t_junction, y_junction, merge, fork | — | 高层道路拓扑模板（非 OpenDRIVE 原生枚举） |
| `type` | enum | 是 | town, lowSpeed, rural, motorway, townArterial, townCollector, townLocal | town | OpenDRIVE 1.5M road type 白名单 |
| `lanes` | object | 是 | `{forward: 1..5, backward: 0..5}` | `{forward:1, backward:1}` | **相对 ego 行驶方向**的车道数。forward=同向（≥1），backward=对向（0=单行道） |
| `center_line` | enum | 是 | solid, broken | broken | 能否跨中心线借对向道（如绕障碍物）。solid=双黄禁跨；broken=可借道。单向时填但无意义 |

## 推理规则（脚本 prompt 内置）

- 事实不明确：`type` 默认 `town`，`lanes` 默认 `{forward:1, backward:1}`，`center_line` 默认 `broken`（保守保留绕行空间）。
- `lanes.forward`/`lanes.backward`：出现 right/left lane、multiple lanes、arterial → 该方向 2；four lanes → 4；five lanes → 5；上限 5。
- **单行道**（明确 one-way street、无对向车道）→ `backward: 0`。
- **不对称**（如北向1条、南向2条）→ forward/backward 取各自实际值。
- `center_line`：double yellow / solid centerline → `solid`；broken / dashed / 允许借道超车 → `broken`。
- 覆盖不了的结构（停车场、私有道、自定义拓扑等）→ 返回 `status: "unsupported"` + 原因，不要硬选最近项。
- 额外输出 `description_zh`（一句中文说明几何与判断依据）。

## 下游可实现性（compiler 现状）

`tools/build_road_seed_opendrive.py` → `replay_scene_tools.write_generated_opendrive` 目前能确定性生成：

- **已实现**：`straight`、`curve`、`cross_intersection`、`t_junction`、`y_junction`、`merge`、`fork`
- **表达力边界（已移出枚举）**：`roundabout`（环岛）—— scenariogeneration 工具链无现成环岛生成器（`JunctionGroup(roundabout)` 仅有数据结构、无端到端生成），真环岛需手搓闭环 road + 多 junction + JunctionGroup，CARLA 导入易碎；语料仅逼出 2/649。决策：不实现，环岛场景由推理器返回 `status: unsupported`。

`curve` 现编译为「直线 stub → arc → 直线 stub」三段链（保证 arc 有前后继 link，CARLA 导入稳定）。
`merge`（匝道汇入 / on-ramp）与 `fork`（匝道分流 / off-ramp）编译为「主路 in + 主路 out + 单车道匝道」经
`DirectJunctionCreator` 直连：merge 的匝道并入下游主路最右车道，fork 由上游主路最右车道分出匝道。

**v2 方向感知的编译范围**：`forward → OpenDRIVE 右车道`（行驶向，负 id），`backward → 左车道`（对向，正 id），
`center_line` 控制中心线 roadmark（solid/broken）。**所有 topology（直路/弯道/cross/t/y/merge/fork）统一应用** `{forward, backward}`：
junction 的每条腿都按同一组 forward/backward 建（`backward=0` 即每条腿单向，已验证可编译）。

## 禁止字段

`road_id` / `lane_id` / `laneLink` / `predecessor` / `successor` / XY / waypoint / actor 轨迹 / collision time / allowed_maneuvers（cross_intersection 已隐含所有合法转向）。

## 示例

```json
{ "topology": "straight", "type": "townLocal", "lanes": { "forward": 1, "backward": 1 }, "center_line": "broken" }
{ "topology": "straight", "type": "town", "lanes": { "forward": 3, "backward": 0 }, "center_line": "broken" }
{ "topology": "cross_intersection", "type": "townArterial", "lanes": { "forward": 2, "backward": 2 }, "center_line": "solid" }
{ "topology": "curve", "type": "rural", "lanes": { "forward": 1, "backward": 1 }, "center_line": "solid" }
```

第二条是**单行三车道**（backward=0）；第三条中心线为双黄实线（禁止借对向道）。

## 下一轮 backlog（本版未做）

- 下游方向感知：`osc_blocks.py` 的 oncoming（对向车）在 `backward=0` 单行道上须禁用；cut_in/adjacent、WF5 约束改按 forward 判定；`api_infer_scene_seed_v2.py` 同步。
- 全量重跑 713 条 road_seed（旧 int `lanes` 产出迁移为 v2）。
- 单向/不对称 junction 现已统一应用同一组 `{forward, backward}` 到每条腿；**每条腿各自不同的车道规格**（异构腿）仍属后续工作。

