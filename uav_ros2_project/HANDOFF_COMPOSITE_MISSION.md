# 复合 Mission 热更新交接文档

## 1. 背景

本项目是 ROS 2 Python + MAVROS 的固定翼无人机任务系统。核心任务入口是：

```text
/fcu/goto_global
```

本次修改前，`fcu_interface_mavros_node.py` 在收到目标后采用“临时任务 + 模式切换 + 原任务恢复”的方式：

```text
原 AUTO mission
  -> 切 GUIDED
  -> clear 当前 mission
  -> push 临时 HOME/A/B/R/[DO_SET_SERVO]/C/D mission
  -> set_current 到 A
  -> 切 AUTO
  -> 等待 D
  -> 恢复原 mission
```

这个流程的工程隐患是：空中切换 `AUTO -> GUIDED -> AUTO`，并且需要在临时 mission 和原 mission 之间来回恢复，状态复杂、失败路径多。

本次改为“方案 A：纯 AUTO 模式下的后台静默拼接与热更新”：

```text
保持当前飞行模式不变
  -> pull 当前原 mission
  -> 找到 break_index
  -> 构建完整复合 mission
  -> clear mission
  -> push 完整复合 mission
  -> pull-back 校验
  -> set_current 到插入的 A 点
```

## 2. 修改文件

```text
src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py
```

本次没有修改 launch、YAML、接口定义和 payload 文件。

## 3. 新 mission 结构

假设原 mission 为：

```text
0 HOME
1 WP1
2 WP2
3 WP3
4 WP4
5 WP5   <- break_index，也就是当前即将执行/当前任务项
6 WP6
...
```

新 mission 会变成：

```text
0 HOME
1 WP1
2 WP2
3 WP3
4 WP4
5 A
6 B
7 R
8 DO_SET_SERVO   可选，只有安全门控允许时插入
9 C
10 D
11 WP5
12 WP6
...
```

如果真实释放关闭，则为：

```text
original[0:break_index]
A
B
R
C
D
original[break_index:]
```

其中：

```text
A 点序号 a_seq = break_index
```

上传完成后会调用：

```text
WaypointSetCurrent(a_seq)
```

## 4. 新增核心函数

### `build_composite_mission(abcdr, original_waypoints, break_index)`

作用：

```text
构建 original prefix + A/B/R/[DO_SET_SERVO]/C/D + original suffix
```

同时会更新：

```text
TEMP_SEQ_A
TEMP_SEQ_B
TEMP_SEQ_R
TEMP_SEQ_RELEASE
TEMP_SEQ_C
TEMP_SEQ_D
```

注意：这些序号现在表示“完整复合 mission 里的位置”，不再是小型临时 mission 里的固定序号。

### `get_current_mission_seq()`

获取当前拼接点 `break_index`：

```text
优先找 is_current == True 的 waypoint
否则使用 /mavros/mission/waypoints.current_seq
```

### async MAVROS 服务封装

新增：

```text
call_service_async()
pull_mission_async()
clear_mission_async()
push_mission_async()
set_current_mission_item_async()
```

这些函数用于保证 MAVROS 服务调用顺序：

```text
pull -> clear -> push -> pull verify -> set_current
```

## 5. `/fcu/goto_global` 新流程

现在 `handle_goto_global()` 的主流程是：

```text
handle_goto_global()
  -> compute_abcdr_points()
  -> 检查 MAVROS connected
  -> 检查 GPS valid
  -> dry_run_goto 检查
  -> allow_mission_upload 检查
  -> handle_goto_global_composite_async()
       -> pull_mission_async()
       -> get_current_mission_seq()
       -> build_composite_mission()
       -> clear_mission_async()
       -> push_mission_async()
       -> pull_mission_async() 做回读校验
       -> verify_temporary_mission()
       -> set_current_mission_item_async(a_seq)
       -> wait_for_current_seq(a_seq)
```

## 6. 主路径已经移除的旧逻辑

新的 `/fcu/goto_global` 主路径不再调用：

```text
self.set_mode('GUIDED')
self.set_mode('AUTO')
upload_temporary_auto_mission()
restart_temporary_auto_mission()
restore_original_auto_mission()
wait_for_c_event()
```

也就是说：

```text
不会为了插入目标任务主动切 GUIDED
不会在 D 点后恢复原 mission
不会再用小型临时 mission 覆盖原任务后再恢复
```

旧函数目前仍保留在文件里，但新主路径已经不再进入它们。后续可以单独做清理。

## 7. 载荷释放逻辑

`DO_SET_SERVO` 仍然走原有安全门控：

```text
control_mode == REAL_CONTROL
enable_real_payload_release == true
servo_channel 合法
safe_pwm 合法
release_pwm 合法
safe_pwm != release_pwm
```

只有门控通过，才会在 R 和 C 之间插入：

```text
MAV_CMD_DO_SET_SERVO
```

否则复合 mission 中只有：

```text
A -> B -> R -> C -> D
```

## 8. 重要行为变化

修改前：

```text
到 D
  -> restore original mission
  -> set resume seq
  -> AUTO
```

修改后：

```text
到 D
  -> 飞控自动继续执行 original suffix
```

原因是原任务后续段已经拼接在 D 后面，不需要恢复原 mission。

## 9. 已完成验证

已完成 Python 语法检查：

```bash
python -m py_compile src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py

python -m compileall -q \
  src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py \
  src/uav_fcu_interface/uav_fcu_interface/mission_verification.py
```

结果：通过。

未完成单元测试，原因是当前 Windows Python 环境没有安装 `pytest`：

```text
No module named pytest
```

## 10. 当前 git 变更摘要

```text
M src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py
M HANDOFF_COMPOSITE_MISSION.md
```

主要代码修改规模：

```text
src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py
323 insertions(+), 267 deletions(-)
```

## 11. 剩余风险

1. 新方案避免了模式切换，但仍然会在空中 clear 并重写完整 mission，必须先做 SITL 验证。
2. 如果 clear 成功但 push 失败，当前主流程不会自动恢复原 mission。
3. `break_index` 当前来自 `is_current` 或 `current_seq`，需要确认 ArduPilot 在 AUTO 中该字段是否总是代表合适拼接点。
4. 旧临时任务、恢复任务、备用航线函数仍在文件中，虽然主路径不用，但后续建议清理或隔离。
5. 当前使用 `asyncio.run()` 包裹 async 服务调用，语法检查通过，但还需要在 ROS 2 executor 下做 SITL 运行验证。
6. `PayloadMonitor` 的释放命令序号需要重新核对，因为 `DO_SET_SERVO` 不再是固定小 mission 序号，而是 `break_index + 3`。
7. B 点检查和备用航线切换逻辑已不在新主路径中执行。如果仍需要 B 点失败切备用航线，需要重新设计为“复合 mission 内部预案”或“再次热更新”。

## 12. SITL 验证建议

1. 启动 SITL 和 MAVROS。
2. 上传一条原始 AUTO mission，至少包含多个航点。
3. 让飞机处于 AUTO 并正在执行原 mission。
4. 发布 `/vision/target_command`。
5. 通过 MissionManager 执行：

```text
/mission/enable
/mission/confirm_target
```

6. 日志中应看到：

```text
composite mission planning started
mission pull requested purpose=composite_source_snapshot
composite mission generated
mission clear requested purpose=composite_update
composite mission push completed
mission pull requested purpose=composite_verification
composite mission pull-back verified
mission current set seq=<break_index> point=A mission_type=COMPOSITE
mission current confirmed seq=<break_index> point=A mission_type=COMPOSITE
```

7. `/fcu/goto_global` 主路径中不应再出现：

```text
flight mode confirmed mode=GUIDED
flight mode confirmed mode=AUTO
restoration started
original mission restored
```

8. Mission Planner / MAVProxy 中应看到 mission 顺序：

```text
original prefix
A
B
R
[DO_SET_SERVO]
C
D
original suffix
```

9. 重点确认：

```text
飞过 D 后继续执行原 mission 后续航点
```

