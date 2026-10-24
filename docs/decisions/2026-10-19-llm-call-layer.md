# 2026-10-19 决策：LLM 调用层（`q2q_harness/llm/`）

> **已被取代（同日）**：用户反馈完整版 6 文件无法消化，现行为**最小版**（只有 `config.py` + `client.py`，
> 输入消息、输出文本）。本文档描述的设计与全部代码在 [`archive/2026-10-19-llm-full/`](../../archive/2026-10-19-llm-full/README.md) 备查；
> 当前学习材料见 `ros2_code/讲解/LLM最小版-两个文件.md`。以下内容保留作为「当初为什么那样设计」的记录。

状态：已实现并离线验证（Stage 1 接口层之上新增的一层）。
上游：[05-最终架构](../../archive/开发基线/05-最终架构.md)（LLM/WM 只输出 Subgoal）。

## 背景

最终架构把「上层规划」画成外部 LLM/WM，Harnes 侧只有 `reorganizer` 这个空回调。
接真实模型前必须先把三件事定下来，否则后面接仿真会反复改：

1. LLM 与 Harness 的**契约**（只给 `Subgoal`，还是给轨迹/动作？）；
2. 失败闭环由谁触发、喂给 LLM 什么上下文；
3. 没有 API key / 断网时如何开发和单测。

## 决策

1. **LLM 只产 `Subgoal`**（`case` + `constraints` + `prompt` + `name`），不产轨迹、关节速度、力矩、求解器或安全阈值。求解器由 `case` 决定（`EmbeddedRollout.CASE_POOL`），LLM 无权指定。
2. **`llm/` 是大文件夹**（与 `policy_arbiter/`、`log/` 同级），6 个文件各管一件事：
   | 文件 | 职责 |
   |---|---|
   | `config.py` | 端点 / 模型 / key / 温度 / 重试 / 离线开关（环境变量 + JSON） |
   | `protocol.py` | `LlmMessage` / `LlmResponse` / `UsageLedger` / `ChatClient` / `EventSink`，无外部依赖、无内部依赖（防循环导入） |
   | `client.py` | `OpenAICompatClient`（urllib，无 requests/openai SDK）+ `MockChatClient`；`build_client()` 是唯一入口 |
   | `prompts.py` | plan / replan / review 三套提示词；case 表**直接取自 `EmbeddedRollout.CASE_POOL`** |
   | `subgoals.py` | LLM 输出 → `Subgoal` 的白名单校验（把 LLM 输出当不可信数据） |
   | `planner.py` | `LlmPlanner`（plan / replan / review）+ `RulePlanner`（离线兜底）+ `Plan` |
3. **失败闭环走 watchdog**：`Q2QHarness.run_task()` 中，`checkpoint` 判 advance；未 advance 时 `watchdog.needs_replan()` 为真 → 调 `planner.replan(..., history=已执行结果, previous=上一份计划)`。`history` 就是架构里 `EpisodeRecorder → LLM` 的那条反馈，只传小 dict（名称/case/成败/错误/policy），不序列化活对象。
4. **输出校验白名单化**：`case` 必须在 `CASE_POOL` 内；`solver` 一律丢弃并记 note；`constraints` 只留数字/字符串/布尔/数字数组；结构错误（无 `subgoals`、空数组、超过 6 个）→ `SubgoalFormatError` → 触发**一次修复回合**（把错误回喂模型重发）。
5. **离线优先**：无 key、网络错误、解析失败 → `RulePlanner` 接管，`Plan.source = "fallback"` 并写 note。没有 key 时 `build_client()` 自动返回 Mock，绝不会误发请求。
6. **单一日志**：`llm_call`（每次调用）、`replan`、`replan_exhausted` 与执行结果写进同一个 `EpisodeRecorder` 文件；`replay` 把前三者当事件统计（`llm_calls` / `llm_call_failures` / `replans`），不影响结果行的成功率。

## 边界与安全

- LLM 的输出永远先过 `subgoals.py`；即使模型输出 `solver`、`force_limit`、嵌套 dict，也进不了下游。
- 危险判定与兜底路由仍在 `Watchdog`（人为硬编码），LLM 不参与安全决策。
- `RulePlanner` 只**放宽**参数（`align_tolerance ×1.5`、`approach_speed ×0.5`），不放宽 case 语义，也不改 `Watchdog` 阈值。

## 配置

优先级：显式参数 > 环境变量 > JSON 文件 > 默认值。

| 项 | 环境变量 | 默认 |
|---|---|---|
| 端点 | `Q2Q_LLM_BASE_URL` | `https://api.deepseek.com/v1` |
| 模型 | `Q2Q_LLM_MODEL` | `deepseek-chat` |
| key | `Q2Q_LLM_API_KEY` / `DEEPSEEK_API_KEY` / `OPENAI_API_KEY` | 空 → 离线 Mock |
| 温度 / 上限 / 超时 / 重试 | `Q2Q_LLM_TEMPERATURE` / `_MAX_TOKENS` / `_TIMEOUT` / `_MAX_RETRIES` | 0.2 / 1200 / 30s / 2 |
| 离线 / JSON 模式 | `Q2Q_LLM_OFFLINE` / `_JSON_MODE` | 0 / 1 |
| JSON 配置路径 | `Q2Q_LLM_CONFIG` | 无（样例：`config/llm.example.json`） |

key 只走环境变量或本地 JSON；`LlmConfig.describe()` 与日志只输出 `sk-1234...abcd`。

## 验证（2026-10-19，本地离线）

- `python -m q2q_harness.llm_demo`（Mock）：4 个子目标 → `align_gripper` 连续两次失败 → watchdog 触发重规划 → `avoid_obstacle`/`realign_gripper`/`retreat` 完成；`success_rate(子目标)=0.83`，`replay` 口径 `attempts=7 / success_rate=0.71`，`llm_calls=3`、`replans=1`、`hardcoded=0`。
- `python -m q2q_harness.llm_demo --fail-llm`：3 次调用全部失败 → `Plan.source=fallback`，规则规划 + 规则重规划仍跑通（`success_rate=0.83`），复盘自动降级为模板复盘。
- `python -m unittest discover -s tests`：25 个用例通过（解析、白名单、配置优先级、修复回合、兜底、重规划预算耗尽、replay 计数）。

## 未决 / 下一步

- 真实模型接入：换 key 跑 `--live`，观察 3 个指标的漂移（子目标数量、case 分布、修复回合次数）。
- 提示词版本化：目前 `prompts.py` 内联；出现 prompt 迭代需求时再抽 `prompts/` 目录 + 版本号。
- `Subgoal` 的跨进程边界：接 `q2q_interfaces` 时要定 `q2q_interfaces/msg/Subgoal.msg`，届时 `llm/` 输出仍应保持这份 dict 结构。
- 视觉输入（`OuterState.vision_tokens`）尚未进 prompt；M6 评测阶段再定 token 化方案。
