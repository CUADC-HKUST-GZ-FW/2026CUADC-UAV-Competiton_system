# Dynamic R：两次完整 mission 上传交付记录

基线：`origin/cjy_0915_修改航线设计`，`75b7caf5b422020c313aa6c1ba738b55ef0b0ed4`（只修改了一半）。

已执行 `git fetch origin`，在本地同名跟踪分支建立独立 worktree：

`C:/Users/ytxia/Desktop/YOUTH_YOLOv26training/github_publish/2026CUADC-cjy0915-full-mission`

附件中的 `C:/Users/yanyan/...` 在本机不存在。原本机统一仓库位于相邻的 `2026CUADC-UAV-Competiton_system`；其 main 和未跟踪文件均保留。本次修改保存在上述 worktree，未部署到设备。

## 1. 当前半成品处理

检查了原仓库和目标分支的 `git status`、`git diff`。原仓库没有已跟踪的未提交修改；目标分支检出后工作区干净。半成品来自分支最后一次提交，而非本机未提交补丁。

| 分类 | 内容 | 处理 |
|---|---|---|
| A：partial 专用 | `mission_partial_adapter.py`、对应测试、原始 MAVLink 发布/订阅、adapter 参数和依赖 | 删除 |
| A：partial 专用 | `push_partial_mission_async()`、单点失败调和/恢复、raw MISSION_CURRENT workaround、旧 R_PUSH/R_PULL 成功链 | 删除，替换为第二次完整 Push/Pull/Verify |
| B：通用 dynamic R | B crossing、冻结快照、TEST ONLY 计算与校验、R_safe、reached deadline、任务锁、验证提交锁、取消机制 | 保留并复用 |
| C：既有代码 | payload、vision、arming、AUTO 启动策略、几何和插入索引算法 | 保留；不回退分支中已完成的 payload 改动 |

原仓库未跟踪内容为 `uav_ros2_project/.gitattributes`、`.gitignore`、`backups/`、`wp/test2 - 副本.waypoints`，以及 `youth-vision-runtime/backups/`、`deploy_0721/`、`diagnostics/` 和两个 `.recording_new` 启动脚本；未移动或删除。

没有使用 `git reset --hard`，没有修改 MAVROS 或 `/opt/ros`。

## 2. 修改文件、函数和原因

以下路径均相对于 `uav_ros2_project`。

| 文件 | 函数/位置 | 修改与原因 |
|---|---|---|
| `src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py` | `handle_goto_global_composite_async()` | 首次完整 Push、Pull、Verify PASS 后才保存私有 safe cache；原启动流程保留 |
| 同上 | `safe_composite_mission`、`build_dynamic_mission()`、`dynamic_mission_structure_matches()` | 防御性深拷贝，只替换 R；其他 item 严格逐字段相等，否则拒绝上传 |
| 同上 | `_dynamic_r_worker()`、`_update_dynamic_r_async()`、`push_mission_async()` | 第二次完整上传仅一次，`start_index=0`；不使用初次上传的 service retry；失败后一次完整回读 |
| 同上 | `_check_dynamic_progress()`、`set_current_mission_item_async()`、`call_service_async()` | 正常进度不 SetCurrent；异常回退才恢复；等待服务后、真正发送前重新检查 deadline 和进度 |
| 同上 | `_commit_dynamic_r_verified()`、`_full_update_event()`、`_dynamic_manual_failure()` | 原子验证提交、完整耗时、四类任务状态与人工处理事件 |
| `src/uav_fcu_interface/uav_fcu_interface/mission_partial_adapter.py` | 全文件 | 删除本轮专用 adapter |
| `src/uav_fcu_interface/uav_fcu_interface/test_mission_partial_adapter.py` | 全文件 | 删除废弃 adapter 测试 |
| `src/uav_fcu_interface/package.xml` | exec dependencies | 删除半成品引入、已不使用的 `mavros` Python helper 与 `python3-pymavlink` 依赖 |
| `src/uav_bringup/launch/real_bringup.launch.py` | FCU 参数字典 | 删除仅供 adapter 使用的 system/component 参数传递；其他节点的对应参数保留 |
| `src/uav_bringup/launch/sitl_mavros_dynamic_abc.launch.py` | FCU 参数字典 | 同上，未启动 SITL |
| `src/uav_bringup/uav_bringup/flight_summary_logger_node.py` | 事件集、metric 字段、`mission_summary_event_callback()`、`_format_human_event()` | 完整上传事件和机器/人类可读摘要；Push 报错后回读成功仍显示 `FAILED (reconciled)`，不伪报 Push PASS |
| `src/uav_mission_manager/uav_mission_manager/mission_manager_node.py` | 新增 summary 订阅与 `dynamic_mission_failure_callback()` | 仅将当前 EXECUTING task 的终局故障接入已有 `fail()/enter_safe()`；状态枚举、转换表和主状态机不变 |
| `src/uav_fcu_interface/uav_fcu_interface/test_abcdr_mission.py` | 原相关测试 | 保留几何、首次上传、deadline 和并发测试；替换/移除废弃 partial 断言；补 safe cache 断言；超时测试改为确定性的过期时间戳 |
| `src/uav_fcu_interface/uav_fcu_interface/test_full_mission_update.py` | 新增 38 个参数展开后的用例 | 完整上传、结构保护、失败调和、进度恢复、取消、实际 request、日志与 SAFE 路由 |
| `scripts/run_unit_tests_without_ros.py` | 显式本地测试入口 | 无 ROS 主机上提供消息/导入替身，运行真实业务函数；不连接任何飞行设备 |
| `docs/DYNAMIC_R_FULL_MISSION_UPDATE.md` | 本报告 | 记录范围、结果和复现方法 |

## 3. 新完整流程

```text
FIRST SAFE FULL PUSH (start_index=0, 完整 R_safe mission)
→ FULL PULL → VERIFY PASS → 保存 safe_composite_mission
→ 原 SetCurrent / AUTO 启动逻辑
→ DYNAMIC TRIGGER → B_STATE_FROZEN → R_CALC_START → R_CALC_DONE
→ deep copy + only R replacement + structure check + deadline check
→ SECOND_FULL_PUSH_START → SECOND_FULL_PUSH_DONE
→ SECOND_FULL_PULL_START → SECOND_FULL_PULL_DONE
→ SECOND_FULL_VERIFY_START → SECOND_FULL_VERIFY_PASS
→ MISSION PROGRESS CHECK
→ DYNAMIC_MISSION_VERIFIED，R_SOURCE=DYNAMIC
```

R 计算失败：`DYNAMIC_UPDATE_SKIPPED reason=r_calculation_failed R_SOURCE=SAFE`，无第二次 Push。上传前 R 已 current/reached：`DYNAMIC_UPDATE_TOO_LATE R_SOURCE=SAFE`，不上传。

第二次开始后不重试、不恢复上传 R_safe；即使取消或软件 deadline 到达，仍允许进行一次只读回读确认。软件 timeout 和 R deadline 均阻止迟到的 dynamic 成功提交。

保留 `dynamic_r_enabled`、`dynamic_r_test_mode`、`dynamic_r_update_timeout_sec`、`aburcd_update_metrics_enabled`。耗时记录包括 trigger、计算起止、第二次 push/pull/verify 起止和持续时间、trigger 到 VERIFIED 总时间，以及更新前后 current/reached。时间戳为单调时钟秒，duration 为毫秒；另保留原事件日志时间。

## 4. 第二次 mission 构造与一致性证据

safe mission 存于私有 tuple，公开属性每次返回深拷贝。dynamic mission 从该缓存复制，仅赋值 `dynamic_mission[r_seq]`，不重新 Pull 原 mission、不重建 prefix/suffix、不重算其他 item。

上传前验证数量相同，并对非 R 的 `frame/command/param1..4/x_lat/y_long/z_alt/autocontinue/is_current` 精确比较。发现任何差异记录 `DYNAMIC_MISSION_STRUCTURE_MISMATCH` 并拒绝上传。

含 RELEASE 的既有配置下，测试证明：A=5、U=6、R=7、RELEASE=8、D=9；逐字段差异集合严格为 **`[7]`**，seq8 的 command/channel/PWM 和 seq9 D 均未改变。真实载荷的既有授权开关保持原状，未强制启用 RELEASE。

FCU 回读继续复用 `verify_mission_waypoints()`，保留既有经纬度/高度编码容差、HOME 高度及 servo frame 归一化规则；回读核验不是序列化字节比较。上传前对本地 safe/dynamic 副本的非 R 比较则是精确比较。

## 5. current_seq 处理

- 上传前记录 `current_seq_before_update`、`last_reached_seq_before_update`。
- 回读验证后读取实时缓存和 reached，记录对应 after 字段；最终提交再持锁检查。
- before=5，after=5 或6，且未到达 R、current 超过 reached：不 SetCurrent。
- 明确回退/落在已 reached item 时，必须有 reached 证据；恢复目标取 before、最新 current、reached+1 的最大值，并要求 `last_reached_seq < resume_seq < r_seq`。
- 等待服务后再次检查：若已自然恢复则不发送 SetCurrent；若 R 已到达则拒绝发送。发送后等待进度确认，不能只凭 service success 认定恢复。
- 无法确定、deadline 已过或恢复未确认：`MISSION_PROGRESS_UNKNOWN`，交给已有 SAFE/人工流程。

R_REACHED、D_REACHED、MISSION_COMPLETE、task cleared、TOO_LATE 均禁止新的第二次 Push。已开始的 worker 最多完成当前服务及一次只读确认，不再尝试上传。

## 6. failure handling

| 回读实际 mission | 结果与处理 |
|---|---|
| 完整匹配 safe | `MISSION_STATE=SAFE`、`R_SOURCE=SAFE`；确认进度正常后继续 safe，不重新上传 |
| 完整匹配 dynamic | `MISSION_STATE=DYNAMIC`；继续 deadline/current progress 检查，通过后才 `DYNAMIC_MISSION_VERIFIED` |
| 均不匹配 | `MISSION_STATE=INCONSISTENT`、`MISSION_INCONSISTENT`；不宣称 safe，进入已有 SAFE/人工处理 |
| Pull 失败 | `MISSION_STATE=UNKNOWN`；不猜测 R_safe 仍存在，进入已有 SAFE/人工处理 |

已确认 dynamic 内容但进度/时限不安全时也进入人工流程，不把它错误标成 SAFE。

现有 MissionManager 的 `enter_safe()` 只阻止后续伴随计算机控制，**不撤销已上传的 FCU mission**，飞控仍可能继续执行；这沿用已有实现，本轮未新增模式切换或飞控中止逻辑。

## 7. 测试结果与限制

| 检查 | 结果 |
|---|---|
| `python -m py_compile`（本机 Python 3.12，8 个修改/新增 Python 文件） | PASS |
| 原 `test_abcdr_mission.py` | 39 PASS |
| 新 `test_full_mission_update.py` | 38 PASS |
| 原日志格式测试 | 10 PASS |
| 原 payload monitor 测试 | 28 PASS |
| 合计 | **115 PASS** |
| `git diff --check` | PASS |
| 受保护算法 AST 对照分支基线 | PASS：dynamic R 公式、候选校验、几何、mission 拼接、payload gate、nav item 构造、首次起点选择未变 |
| `colcon build` | NOT RUN：Windows 主机无 ROS 2/colcon；WSL 仅 docker-desktop |
| 原需实例化 ROS 节点的 MissionManager 集成测试 | NOT RUN：缺 ROS 2；新增故障回调业务单测已通过 |
| SITL | 未启动 |

复现无 ROS 单测（先在所用 Python 环境安装 pytest）：

```text
python scripts/run_unit_tests_without_ros.py -q
```

此测试入口只替换 ROS 消息/导入和由各用例明确替换的服务边界；实际 FCU/MAVROS 传输、ROS QoS/调度、整机运行未验证，115 PASS 不是实飞验收。

本机验证记录位于：

`C:/Users/ytxia/Desktop/YOUTH_YOLOv26training/diagnostics/20260916_dynamic_r_full_mission_update/validation.json`

同目录 `unit-tests.xml` 保存最终 115 项测试结果。

几何参数未修改。分支代码中 D 默认偏移为160米，附件描述最近实测为50米；本轮保留分支原值。没有依据单元测试推断 B 点剩余时间足够；后续仍需真实运行耗时决定是否调整 B。
