# ABURCD 动态 R 任务交接说明

最后核对日期：2026-09-15

适用环境：ROS 2 Humble、MAVROS、ArduPlane、Python 3

## 1. 本阶段目标与结论

本阶段只验证一件事：飞机经过虚拟 B 后，机载电脑能否在 R 成为
`MISSION_CURRENT` 之前，只更新未来的 R 航点并完成飞控回读验证。

几何关系为：

```text
A → B（虚拟）→ U → R → C（虚拟）→ D
```

实际上传给 FCU 的任务为：

```text
A → U → R_safe → DO_SET_SERVO → D
```

其中：

- B、C 仍然只是几何点，没有 MAVLink waypoint seq；
- U 是新增的真实 `NAV_WAYPOINT`；
- 首次完整上传始终使用原有固定安全点 `R_safe`；
- 飞过 B 后才尝试把 R 单点更新成 `R_dynamic`；
- 动态更新失败时不得破坏整条任务；
- `DO_SET_SERVO` 的控制方式、通道和 PWM 没有改变；
- 正式动态 R 物理公式尚未实现；
- 当前动态公式仅供 SITL 使用，并且必须显式打开 TEST ONLY 模式。

## 2. 默认 seq 示例

默认参数：

```text
insert_wp_index = 5
resume_wp_index = 10
DO_SET_SERVO = enabled
```

修改前实际任务段：

```text
seq 5  A
seq 6  R_safe
seq 7  DO_SET_SERVO
seq 8  D
seq 9  原任务恢复段
```

修改后实际任务段：

```text
seq 5   A
seq 6   U
seq 7   R_safe / R_dynamic
seq 8   DO_SET_SERVO
seq 9   D
seq 10  原任务恢复段
```

实际 seq 不写死。任务拼接后统一保存在：

```python
dynamic_indices = {
    'A': ...,
    'U': ...,
    'R': ...,
    'RELEASE': ...,
    'D': ...,
    'RESUME': ...,
}
```

B、C 不会出现在 `dynamic_indices` 中，这是有意设计，不是遗漏。

## 3. 关键代码位置

### 3.1 FCU 与动态更新

文件：

```text
uav_ros2_project/src/uav_fcu_interface/uav_fcu_interface/
fcu_interface_mavros_node.py
```

类：

```text
FcuInterfaceMavrosNode
```

关键函数：

| 函数 | 职责 |
|---|---|
| `compute_abcdr_points()` | 生成 A/B/U/R/C/D 几何点 |
| `build_composite_mission()` | 拼接 A/U/R_safe/RELEASE/D 和原任务 |
| `waypoints_callback()` | 独立跟踪 `MISSION_CURRENT` |
| `waypoint_reached_callback()` | 独立跟踪 `WAYPOINT_REACHED` |
| `_virtual_b_signed_distance_m()` | 计算飞机位于虚拟 B 前还是后 |
| `_maybe_detect_virtual_b_crossing()` | 判定 B 穿越并冻结快照 |
| `compute_dynamic_r()` | 动态 R 算法接口 |
| `validate_dynamic_r_candidate()` | 检查候选 R 是否位于 U—C 合法区间 |
| `_dynamic_r_worker()` | 在后台线程执行计算和 mission transaction |
| `_update_dynamic_r_async()` | partial push、pull-back 和完整验证 |
| `push_partial_mission_async()` | 只上传一个 R waypoint |
| `pull_mission_async()` | 从 FCU 重新拉取完整任务 |
| `_restore_safe_r_once_async()` | 验证失败时单次恢复 R_safe |

### 3.2 日志

文件：

```text
uav_ros2_project/src/uav_bringup/uav_bringup/
flight_summary_logger_node.py
```

沿用现有 `FlightSummaryLoggerNode`，没有另建独立 logger。

### 3.3 参数与 launch

```text
uav_ros2_project/src/uav_bringup/uav_bringup/launch_params.py
uav_ros2_project/src/uav_bringup/launch/real_bringup.launch.py
uav_ros2_project/src/uav_bringup/launch/sitl_mavros_dynamic_abc.launch.py
```

### 3.4 测试

```text
uav_ros2_project/src/uav_fcu_interface/uav_fcu_interface/
test_abcdr_mission.py
```

## 4. B 的触发规则

B 不是“航点到达事件”。它由 GPS 轨迹穿越检测产生。

以 A 为原点，以 A→B 为正方向：

```text
signed_distance = 飞机位置在 A→B 上的投影距离 - AB 长度
```

只有观察到：

```text
previous_signed_distance < 0
current_signed_distance >= 0
```

才认为穿越 B。

此外必须同时满足：

1. `dynamic_r_enabled=true`；
2. 当前任务类型是 `COMPOSITE`；
3. 几何点和 seq 索引已经生成；
4. 本次任务尚未触发过 B；
5. 已收到至少两个可以形成“B 前→B 后”的位置样本；
6. `MISSION_CURRENT == u_seq`；
7. `MISSION_CURRENT < r_seq`；
8. mission waypoint list 可用，`current_seq` 不是 `None`；
9. mission transaction 没有被其他上传占用。

融合 GPS 和原始 GPS 回调共用互斥锁，防止两个回调同时启动两次更新。

### 4.1 当前 B 判定不包含的条件

以下条件本阶段没有加入：

- 距离 B 小于某个半径；
- B 横向偏差门限；
- 航向误差门限；
- 地速稳定门限；
- roll/pitch 稳定窗口；
- AB 稳定性确认。

现有 AB trajectory checker仍然是观察用途，不会阻止动态 R 更新。
飞机只要穿过 B 的垂直截面，即使横向偏差较大，也可能触发。

### 4.2 初始位置边界

如果节点收到的第一个位置已经位于 B 后方，不会立即触发。系统需要实际观察到
一次负值到非负值的变化。因此 SITL 不应从 B 后方直接开始测试。

## 5. B 状态快照

触发后发布：

```text
[ABURCD] B_CROSSED
[ABURCD] B_STATE_FROZEN
```

快照包含：

```text
timestamp_unix_sec
timestamp_monotonic
current_seq
lat
lon
altitude_m
ground_speed_mps
vertical_speed_mps
heading_deg
distance_to_u_m
distance_to_r_m
snapshot_latency_ms
```

数据来源：

- 经纬度和高度：最新 GPS；
- 地速、垂直速度、航向：最新 `/mavros/vfr_hud`；
- mission current：最新 `/mavros/mission/waypoints.current_seq`。

目前采用最新缓存，没有对 GPS 与 VFR HUD 做严格时间同步，也没有在 B 触发器中
新增传感器数据年龄检查。

## 6. 动态 R 算法状态

正式动态 R 公式尚未提供，代码不会自行发明。

默认参数：

```text
dynamic_r_enabled=true
dynamic_r_test_mode=true
```

这是当前 SITL 验证阶段的 TEST ONLY 默认配置，不代表正式动态 R 公式已经实现。

三种行为：

| 配置 | 行为 |
|---|---|
| `dynamic_r_enabled=false` | 不触发 B 动态更新，继续使用 R_safe |
| enabled=true、test=false | 返回 `production_dynamic_r_formula_not_configured`，保留 R_safe |
| enabled=true、test=true | 使用 TEST ONLY 固定偏移生成候选 R |

TEST ONLY 候选 R 还必须满足：

- 经纬度、高度都是有限数值；
- 位于 U 和 C 之间；
- 不等于 U 或 C；
- 相对 U—C 直线的横向偏差不超过 1 m。

否则记录：

```text
[ABURCD] R_DYNAMIC_OUT_OF_RANGE
```

## 7. Partial mission update

使用 MAVROS 服务：

```text
/mavros/mission/push
mavros_msgs/srv/WaypointPush
```

请求必须为：

```text
start_index = dynamic_indices['R']
waypoints = [dynamic_r_waypoint]
```

成功条件：

```text
response.success == true
response.wp_transfered == 1
```

禁止在 B 触发后重新上传整条 composite mission。

## 8. 回读验证与 fallback

partial push ACK 不是最终成功。

ACK 后继续调用 `/mavros/mission/pull`，等待新的 waypoint list，并验证：

1. mission 数量不变；
2. R seq 不变；
3. R 经纬度和高度变成候选值；
4. A、U 没有变化；
5. `DO_SET_SERVO` 命令和参数没有变化；
6. D 没有变化；
7. 原任务 prefix/suffix 没有变化；
8. 验证完成时 `current_seq < r_seq`。

成功后记录：

```text
[ABURCD] R_PUSH_VERIFIED
```

验证失败时只尝试一次 R_safe partial restore，再次 pull 并验证。不会无限循环恢复。

如果 R 已经成为 current，禁止继续修改或恢复：

```text
[ABURCD] R_UPDATE_REJECTED_LATE
[ABURCD] ERROR_R_ACTIVE_BEFORE_UPDATE_VERIFIED
```

## 9. 正常事件顺序

由于 B 只有在 U 已是 current 时才能触发，所以正常顺序应理解为：

```text
MISSION_CURRENT_A
→ MISSION_CURRENT_U
→ B_CROSSED
→ B_STATE_FROZEN
→ R_CALC_START
→ R_CALC_DONE
→ R_PUSH_START
→ R_PUSH_ACK
→ R_PUSH_VERIFIED
→ MISSION_CURRENT_R
```

`WAYPOINT_REACHED` 和 `MISSION_CURRENT` 是两套不同的 MAVROS 事件，不保证在日志中
完全相邻。唯一必须满足的核心不变量是：

```text
R_PUSH_VERIFIED 的时间 < MISSION_CURRENT_R 的时间
```

否则本次动态更新判定为太晚。

## 10. 指标和 `null`

每次运行目录新增：

```text
~/uav_flight_logs/<mode>/<run_id>/aburcd_update_metrics.jsonl
```

主要指标：

```text
snapshot_latency_ms
calc_duration_ms
push_duration_ms
verify_duration_ms
dynamic_update_total_ms
r_commit_margin_sec
r_commit_margin_m
```

`r_commit_margin` 的定义是：

```text
R_PUSH_VERIFIED → MISSION_CURRENT_R
```

它只在 `MISSION_CURRENT_R` 时计算，不使用更晚的 `R_REACHED` 覆盖。

以下 `null` 属于正常情况：

- B、C 没有 seq；
- 非 reached 事件的 `reached_seq=null`；
- 成功事件的 `failure_reason=null`；
- 某个事件不适用的耗时字段为 `null`；
- 尚未收到 GPS 时，位置和距离字段为 `null`；
- 尚未收到 VFR HUD 时，速度、垂直速度和航向为 `null`；
- 尚未验证 R 时，commit margin 为 `null`；
- 禁用释放命令时，`release_seq=null`。

仅在 dynamic R 启用时输出简化的成功/失败摘要；禁用时不输出
`ABURCD UPDATE SUMMARY ... NOT OBSERVED`。

## 11. 当前临时距离与风险

当前默认值：

```text
a_offset_m = 160
b_offset_m = 110
u_offset_m = 80       # TEMPORARY TEST VALUE
release_offset_m = 56
d_offset_m = 50
u_acceptance_radius_m = 15
```

因此：

```text
BU 中心点距离 = 30 m
UR 中心点距离 = 24 m
```

巡航速度约 23 m/s 时，飞完 30 m 约 1.3 秒。U 现在使用独立的 15 m 接受半径，
ArduPlane 还可能提前切换 `MISSION_CURRENT_R`，所以真实可用更新时间可能比 1 秒更短。

这组参数只能用于测量和暴露 `UPDATE_TOO_LATE`，不能当作实飞安全距离。
`dynamic_r_update_timeout_sec=6` 也不代表飞机会为更新等待 6 秒；飞机不会暂停。

## 12. 编译与单元测试

进入 ROS 工作区：

```bash
cd ~/uav_ros2_ws
source /opt/ros/humble/setup.bash
```

构建：

```bash
colcon build --symlink-install --packages-select \
  uav_interfaces \
  uav_fcu_interface \
  uav_bringup \
  uav_payload \
  uav_mission_manager \
  uav_vision_bridge
```

语法检查：

```bash
python3 -m py_compile \
  src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py \
  src/uav_fcu_interface/uav_fcu_interface/test_abcdr_mission.py \
  src/uav_bringup/uav_bringup/flight_summary_logger_node.py \
  src/uav_bringup/uav_bringup/launch_params.py \
  src/uav_bringup/launch/real_bringup.launch.py \
  src/uav_bringup/launch/sitl_mavros_dynamic_abc.launch.py
```

专项测试：

```bash
source install/setup.bash
python3 -m pytest -q \
  src/uav_fcu_interface/uav_fcu_interface/test_abcdr_mission.py
```

2026-09-15 最后一次结果：

```text
py_compile: PASS
ABURCD 专项测试: 37 passed
相关功能测试: 102 passed
colcon build: 6 packages finished
git diff --check: PASS
```

专项测试已覆盖：正常单点更新、计算越界、push 失败、verify 失败、timeout、
R 已成为 current、mission busy，以及单次 R_safe 恢复。

## 13. SITL 前置检查

### 13.1 GeographicLib

MAVROS 必须能读取：

```text
/usr/share/GeographicLib/geoids/egm96-5.pgm
```

检查：

```bash
test -r /usr/share/GeographicLib/geoids/egm96-5.pgm \
  && echo READY \
  || echo MISSING
```

如果缺失：

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

截至 2026-09-15，开发机检查结果仍为 `MISSING`。实际尝试中 ArduPlane 可以启动，
但 MAVROS 因 GeographicLib exception 退出，MissionManager 停留在 `WAIT_FCU`。

### 13.2 当前仓库测试文件

```text
uav_ros2_project/param/real_plane_sitl_safe_plane.param
uav_ros2_project/wp/test2.waypoints
uav_ros2_project/test_data/vision_result/target_001/result.json
```

现有 `start_uav_sitl_4pane.sh` 中部分 `/home/yanyan/...` 硬编码路径在开发机上
不存在。使用脚本前应先改为本机实际路径，或者手动启动。

## 14. SITL 动态 R 启动参数

当前公共默认值已经打开两层开关；下面的显式参数便于启动时复核：

```bash
ros2 launch uav_bringup sitl_mavros_dynamic_abc.launch.py \
  dynamic_r_enabled:=true \
  dynamic_r_test_mode:=true \
  u_offset_m:=80.0 \
  u_acceptance_radius_m:=15.0 \
  dynamic_r_test_offset_m:=50.0
```

当前 `dynamic_r_test_mode=true` 只用于验证阶段。正式实飞前必须显式关闭，
直到生产动态 R 公式完成评审。

启动后先确认日志打印：

```text
dynamic_r_enabled=true
dynamic_r_test_mode=true
u_offset_m=80.0
dynamic_r_update_timeout_sec=6.0
```

然后确认：

```text
/mavros/state.connected == true
MissionManager 已离开 WAIT_FCU
原始 mission 已加载
飞机处于允许插入任务的 seq 窗口
```

## 15. SITL 验收清单

### 15.1 正常链路

- composite mission 中只有 A/U/R/RELEASE/D；
- B、C 没有真实 seq；
- `MISSION_CURRENT_U` 发生后才能出现 `B_CROSSED`；
- partial push 的 `start_index` 等于 R seq；
- `wp_transfered == 1`；
- pull-back mission 总数不变；
- `DO_SET_SERVO` 仍在原 seq；
- `R_PUSH_VERIFIED` 早于 `MISSION_CURRENT_R`；
- metrics JSONL 中关键耗时不是 `null`；
- 最终结果为 `DYNAMIC_R`。

### 15.2 失败链路

至少验证：

- 动态 R 计算失败；
- 候选 R 超出 U—C；
- partial push 失败；
- pull/verify 失败；
- update timeout；
- R 已经成为 current；
- mission transaction busy。

共同验收要求：

- 节点不崩溃；
- mission 数量不变；
- A/U 不变；
- `DO_SET_SERVO`、D 和原任务恢复段不变；
- 明确输出失败原因；
- 能确认安全点时结果为 `R_SAFE_FALLBACK`；
- 无法确认恢复时不得宣称已经恢复。

## 16. 实飞前禁止事项

在以下工作完成前，不允许把 TEST ONLY 动态 R 用于实飞：

1. 实现并评审正式动态 R 物理公式；
2. 完成多轮 SITL 正常和失败场景测试；
3. 统计 `dynamic_update_total_ms` 的 P95/P99；
4. 统计 `r_commit_margin_sec/m`；
5. 重新确定 BU 和 UR；
6. 考虑 U 接受半径造成的提前切换；
7. 验证 NX 与飞控之间真实链路延迟；
8. 正式实飞时显式设置并确认 `dynamic_r_test_mode=false`（当前公共默认值为 true）。

本阶段没有处理：

- AB 最终长度；
- CD 最终长度；
- AB/CD 稳定性数据采集；
- 命中优化算法；
- payload servo 控制逻辑；
- `DO_SET_SERVO` 参数；
- MissionManager 的 STANDBY/EXECUTING 安全业务。

## 17. 下一位开发者建议顺序

1. 安装 GeographicLib geoid 数据；
2. 修正本机 SITL 脚本路径；
3. 保持 TEST ONLY 模式完成第一次正常 SITL；
4. 保存 `mission_summary.log`、`all_nodes.log` 和
   `aburcd_update_metrics.jsonl`；
5. 验证 `R_PUSH_VERIFIED < MISSION_CURRENT_R`；
6. 重复测试并统计更新时间；
7. 再讨论 BU/UR 距离；
8. 最后才接入正式动态 R 公式。
