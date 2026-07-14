# Signature schema (sig_v1)

> 用途：聚类/saturation 专用的"轻量分类签名"，与完整 `scene_seed` 解耦。
> 仅锁结构性 4 维（topology/lanes 从已全量的 `road_seed` 拼），不输出
> `params` / `description_zh` / `evidence` / `environment` 等下游字段，
> 因此 schema 比 `scene_seed_schema_v2.md` 稳定得多——预期长期不变。

## schema_version

`sig_v1` — 任何下面四维的取值集合或语义变更都必须 bump 版本号，缓存按版本号失效。

## 输出 JSON 形态

```jsonc
{
  "schema_version": "sig_v1",
  "case_id": "021_Zoox_Inc._March_28_2025",
  "status": "supported|skipped|needs_extension",

  // status=supported 时必填，其余两态为 null
  "signature": {
    "sut_maneuver":   "straight|left|right|overtake_oncoming",
    "npc_count":      0,
    "npc_sig":        ["<position>|<behavior_block>", ...],   // 集合，输出时排序去重
    "collision_pair": "ego↔v1|ego↔v2|ego↔av|ego↔debris|ego↔<other>|<a>↔<b>"
  },

  "reason": "...",      // skipped/needs_extension 时填一句中文
  "pipeline": {
    "stage": "signature_v1",
    "model": "deepseek/deepseek-v4-pro",
    "prompt_sha16": "...",
    "pdf_sha16":    "..."
  }
}
```

## 三态语义（与 scene_seed_v2 一致）

| status | 何时 | signature 字段 |
|---|---|---|
| `supported` | 能用现有 position+behavior 词表表达 | 必填 |
| `skipped` | 倒车 / 泊车 / 单车冲出路缘 / 无第二参与者 / 全静止 / 非碰撞 | null |
| `needs_extension` | 真双方碰撞但词表表达不了，需离线扩词 | null |

`reason` 在后两态必须填一句话，便于人工审计。

## 受控词表（与 scene_seed_v2 同源，sig_v1 锁定）

引用 `tools/api_infer_scene_seed_v2.py` 顶部的常量，**任何枚举增删都视为 schema 演化**：

- **SUT_MANEUVERS** = `{straight, left, right, overtake_oncoming}` —— ego 必须移动
- **POSITIONS_BY_KIND**（每个 NPC 的 position 必须出自其 kind 允许集）：
  - vehicle: `ahead_same_lane / behind_same_lane / adjacent / oncoming / cross / opposing_leg`
  - cyclist: `adjacent / oncoming / cross / roadside`
  - pedestrian: `roadside / ahead_same_lane`
  - static: `ahead_same_lane / roadside`
- **BLOCKS_BY_KIND**（每个 NPC 的 behavior.block 必须出自其 kind 允许集）：
  - vehicle: `front_brake / rear_hit / cut_in / oncoming / stopped_ahead / static_hold / junction_cross / junction_turn / light_change_start`
  - cyclist: `cut_in / oncoming / junction_cross / cross / light_change_start`
  - pedestrian: `cross / walk_along / light_change_start`
  - static: `static_block`

## npc_sig 形成规则

每个 NPC 贡献一个字符串 `f"{position}|{behavior_block}"`，所有 NPC 字符串去重排序后构成集合。
聚类距离对该维做 Jaccard（与 `cluster_eval_set._hamming` 第 4 维处理一致）。
NPC 的 `kind` 与 `side` 不进 signature（与现行聚类口径一致）。

## collision_pair 规则

- 若一端是 `ego` → `f"ego↔{other_id}"`，例如 `ego↔v2`、`ego↔debris`
- 若两端都不是 ego → `f"{a}↔{b}"`（按字母序），出现频率应极低
- 若 `collision` 缺失或两 actor 任一无效 → `"-"`（视作 skipped 候选）

## 显式排除项（与 scene_seed_v2 注释一致）

- `scene.environment`（weather / time_of_day / friction_scale）—— mutation knob，不进结构性签名
- 各 block 的 `params`（speed / trig_ttc / brake_t ...）—— 下游变异空间，不进签名
- `description_zh` / `evidence` / `source_snippets` —— 仅完整 scene_seed 需要

## 字段拼接（聚类时）

聚类的 6 维特征向量 = signature 的 4 维 + road_seed 的 2 维：

```
(road.topology, _bin_lane(road.lanes), sig.sut_maneuver, _bin_npc_count(sig.npc_count),
 tuple(sorted(sig.npc_sig)), sig.collision_pair)
```

`road_seed` 已全量在 `outputs/road_seed/`，`signature` 全量在 `outputs/signature/`，
`cluster_eval_set.py --signature-dir` 走该路径，与原 `--scene-dir` 路径产出等价。

## 缓存键

```
key = sha256(pdf_text)[:16] + "_" + sha256(system_prompt)[:16] + "_" + model_slug + "_" + schema_version
```

- `pdf_text` = `outputs/extracted_text/<case_id>.txt` 的 UTF-8 bytes
- `system_prompt` = 当前 `tools/api_infer_signature.system_prompt()` 输出的 UTF-8 bytes
- `model_slug` = `args.model`（含 `/`，转 `_`）
- `schema_version` = `sig_v1`

缓存写入 `outputs/signature_cache/<key>.json`，索引文件 `outputs/signature_cache/_index.json`
反查 `case_id → key`，便于按 case_id 删除/重跑。
