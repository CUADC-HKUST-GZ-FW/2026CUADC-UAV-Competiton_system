# 固定翼无人机 ROS2 临时任务控制系统

## 1. 项目简介

本项目用于固定翼无人机在执行原始 AUTO 航线的过程中，根据外部输入的目标点坐标和期望航向，临时生成一段 A-B-C-D 任务航线，使飞机飞过目标点 C。临时任务执行结束后，系统会恢复原始 AUTO 航线，并从指定航点继续执行。

当前版本暂未接入视觉模块，目标点坐标和航向通过 ROS2 命令手动发布。后续视觉模块接入后，只需要替代手动发布目标点的步骤。

---

## 2. 当前系统功能

当前版本实现了以下功能：

1. 在原始 AUTO 航线中飞行；
2. 接收目标点 C 的经纬度和期望通过航向；
3. 等待原 AUTO 航线到达指定航点后，再执行临时任务；
4. 根据目标点和航向自动计算临时 A-B-C-D 航点；
5. 清空当前 mission，上传临时 AUTO mission；
6. 执行临时任务并监测是否经过目标点 C；
7. 如果飞控判断已通过 C，但代码 GPS 判断未达到 C，可自动重飞一次临时任务；
8. 无论成功、失败、超时或异常，均恢复原始 AUTO mission；
9. 恢复原始 mission 后，从指定航点继续执行。

---

## 3. 系统结构

```text
手动输入目标点 / 后续视觉模块
        ↓
/vision/target_command
        ↓
mission_manager_node
        ↓
/fcu/goto_global
        ↓
fcu_interface_mavros_node
        ↓
MAVROS2
        ↓
ArduPilot / ArduPlane
```

当前核心节点：

```text
uav_interfaces
    自定义消息和服务接口

uav_mission_manager
    接收目标点，调用飞控接口服务

uav_fcu_interface
    负责 MAVROS 通信、临时 mission 上传、原 mission 备份与恢复

uav_bringup
    存放 launch 文件
```

---

## 4. 主要话题和服务

### 4.1 目标点输入话题

```text
/vision/target_command
```

消息类型：

```text
uav_interfaces/msg/TargetCommand
```

字段：

```text
float64 latitude
float64 longitude
float64 heading_deg
```

含义：

```text
latitude     目标点 C 的纬度
longitude    目标点 C 的经度
heading_deg  期望飞机飞过目标点的航向角
```

航向角定义：

```text
0°   = 北
90°  = 东
180° = 南
270° = 西
```

---

### 4.2 飞控任务服务

```text
/fcu/goto_global
```

由 `mission_manager_node` 调用，最终触发临时 AUTO mission 逻辑。

---

## 5. 临时 A-B-C-D 航线逻辑

输入目标点 C 和航向 `heading_deg` 后，系统自动计算：

```text
A = C 点沿反航向方向后退 a_offset_m
B = C 点沿反航向方向后退 b_offset_m
C = 目标点
D = C 点沿航向方向前进 d_offset_m
```

临时 mission 为：

```text
A → B → C → D
```

当前默认参数示例：

```text
A 距离 C：160 m
B 距离 C：80 m
D 距离 C：160 m
```

如果输入：

```text
C = 22.88480535, 113.49559465
heading_deg = 270°
```

则表示希望飞机从东向西飞过目标点 C：

```text
东侧                         西侧
A -------- B -------- C -------- D
                  目标点
```

---

## 6. 关键参数说明

参数位于：

```text
uav_bringup/launch/sitl_mavros_dynamic_abc.launch.py
```

### 6.1 原 AUTO 航线触发与恢复参数

```python
{'start_after_original_wp_seq': 5},
{'restore_original_wp_seq': 9},
```

含义：

```text
start_after_original_wp_seq
    收到目标点后，不立即执行临时任务；
    等原 AUTO mission 到达该航点后，才开始执行临时 A-B-C-D mission。

restore_original_wp_seq
    临时任务结束后，恢复原始 AUTO mission；
    并从该航点继续执行。
```

注意：MAVROS 的 mission 序号通常从 0 开始。

---

### 6.2 临时任务几何参数

```python
{'a_offset_m': 160.0},
{'b_offset_m': 80.0},
{'d_offset_m': 160.0},
```

含义：

```text
a_offset_m
    A 点距离目标点 C 的距离，方向为反航向。

b_offset_m
    B 点距离目标点 C 的距离，方向为反航向。

d_offset_m
    D 点距离目标点 C 的距离，方向为航向方向。
```

---

### 6.3 航点接受半径

```python
{'a_acceptance_radius_m': 30.0},
{'b_acceptance_radius_m': 15.0},
{'c_acceptance_radius_m': 8.0},
{'d_acceptance_radius_m': 30.0},
```

这些参数会写入临时 AUTO mission 的 `MAV_CMD_NAV_WAYPOINT` 参数中，用于飞控判断航点是否完成。

其中 `c_acceptance_radius_m` 同时也用于代码侧判断飞机是否足够接近目标点 C。

---

### 6.4 临时任务高度

```python
{'mission_altitude_m': 35.0},
```

当前建议先保持与原始 AUTO 航线高度一致，便于验证水平航迹逻辑。后续路径稳定后，再测试降低高度。

---

### 6.5 重飞和超时参数

```python
{'wait_c_timeout_sec': 180.0},
{'max_temp_mission_attempts': 2},
```

含义：

```text
wait_c_timeout_sec
    每一遍临时任务中，等待飞机接近 C 点的最长时间。

max_temp_mission_attempts
    临时任务最多执行次数。
    当前为 2，表示第一遍失败后最多重飞一次。
```

---

## 7. 编译

在工作空间根目录执行：

```bash
cd ~/uav_ros2_ws
colcon build
source install/setup.bash
```

如果修改了自定义接口，例如 `TargetCommand.msg` 或 `GoToGlobal.srv`，建议完整清理后重新编译：

```bash
cd ~/uav_ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

---

## 8. SITL 测试流程

### 8.1 启动 ArduPlane SITL

```bash
cd ~/ardupilot
sim_vehicle.py --kill
sim_vehicle.py -v ArduPlane --console --map --custom-location=22.8847501,113.4961007,20,270
```

---

### 8.2 启动 MAVROS2

```bash
cd ~/uav_ros2_ws
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14555
```

---

### 8.3 启动任务系统

```bash
cd ~/uav_ros2_ws
source install/setup.bash
ros2 launch uav_bringup sitl_mavros_dynamic_abc.launch.py
```

---

### 8.4 发布目标点和航向

当前版本暂未接入视觉模块，因此使用命令行手动发布目标：

```bash
ros2 topic pub --once /vision/target_command uav_interfaces/msg/TargetCommand "{
  latitude: 22.88480535,
  longitude: 113.49559465,
  heading_deg: 270.0
}"
```

发布后，系统不会立即执行临时任务，而是先等待原始 AUTO mission 到达 `start_after_original_wp_seq` 设置的航点。

---

## 9. 正常运行日志参考

收到目标点后：

```text
GOTO_GLOBAL received. Temporary AUTO mission with delayed trigger/restore/retry will be used.
Computed temporary ABCD points ...
Waiting for original AUTO mission to reach wp_seq=5
```

原 AUTO 到达触发航点后：

```text
Original AUTO trigger reached
Temporary AUTO mission uploaded
AUTO mode
Attempt 1: monitoring C
```

如果代码判断到达 C：

```text
C reached by GPS condition
Restoring original AUTO mission
Original AUTO mission restored and AUTO mode resumed from wp_seq=9
```

如果飞控判断通过 C，但 GPS 判断未到达 C：

```text
FCU has passed C, but GPS condition was not met
Restarting temporary mission
Attempt 2: monitoring C
```

如果第二次仍失败或超时：

```text
Temporary mission failed
Original AUTO mission restored and AUTO mode resumed from wp_seq=9
```

---

## 10. 当前版本限制

1. 当前版本会临时清空飞控中的 mission，并上传 A-B-C-D 临时 mission；
2. 执行结束后会重新上传原始 mission；
3. 恢复原始 mission 后，会从 `restore_original_wp_seq` 指定的航点继续；
4. 当前版本暂未接入真实视觉模块；
5. 目标点由命令行手动发布；
6. 当前恢复逻辑依赖 MAVROS mission pull/push，实机测试前必须充分验证；
7. mission 序号通常从 0 开始，需要结合 Mission Planner 和 MAVROS 日志确认实际序号。

---

## 11. 后续如何接入视觉模块

后续视觉模块只需要替代当前手动发布目标点的命令。

也就是说，视觉模块不需要直接控制飞控，不需要直接调用 MAVROS。它只需要发布：

```text
/vision/target_command
```

消息类型：

```text
uav_interfaces/msg/TargetCommand
```

例如在视觉节点中：

```python
from uav_interfaces.msg import TargetCommand

msg = TargetCommand()
msg.latitude = target_lat
msg.longitude = target_lon
msg.heading_deg = target_heading_deg

publisher.publish(msg)
```

---

## 12. 推荐的视觉模块接入路线

### 第一阶段：假视觉节点

先写一个 `mock_vision_node`，在固定延时或固定条件下自动发布目标点。

作用是验证：

```text
视觉输出 → mission_manager → fcu_interface → 临时 mission → 恢复原 AUTO
```

---

### 第二阶段：图案识别 + 坐标查表

视觉模块识别天井图案或目标编号，例如：

```text
target_id = A
confidence = 0.86
```

然后查表得到目标 GPS 和通过航向：

```python
TARGET_TABLE = {
    "A": {
        "latitude": 22.88480535,
        "longitude": 113.49559465,
        "heading_deg": 270.0,
    },
    "B": {
        "latitude": 22.88490000,
        "longitude": 113.49530000,
        "heading_deg": 180.0,
    },
}
```

识别稳定后发布 `/vision/target_command`。

---

### 第三阶段：真实图像定位

如果视觉模块需要从图像像素直接计算目标 GPS，则需要增加：

```text
相机内参
相机外参
飞机姿态
飞机当前 GPS
飞行高度
地面平面假设
像素坐标到地理坐标转换
```

该方案更完整，但开发难度更高，建议在前两阶段稳定后再做。

---

## 13. 建议视觉模块输出字段扩展

当前消息只有：

```text
latitude
longitude
heading_deg
```

后续可考虑扩展为：

```text
float64 latitude
float64 longitude
float64 heading_deg
float64 confidence
string target_id
```

这样便于判断：

```text
识别可信度
目标编号
是否重复触发
是否为同一个目标
```

---

## 14. 安全建议

1. 所有新逻辑先在 SITL 中测试；
2. 实机前确认原始 AUTO mission 能被正确保存和恢复；
3. 实机前确认 `start_after_original_wp_seq` 和 `restore_original_wp_seq` 对应正确航点；
4. 视觉模块不要连续高频发布目标，应加入置信度阈值和去抖；
5. 临时 mission 参数应先使用较大点距和较宽半径，确认流程稳定后再逐步收紧。
# Payload release observability

`payload_monitor_node` is a read-only ROS 2 observer for the payload-release
mission item. It subscribes to the FCU mission, reached-waypoint, RC output and
MAVROS state topics. It has no MAVROS service client and does not upload a
mission, change flight mode, or write a servo value.

The monitor emits three task-correlated milestones:

```text
[PAYLOAD][task=target_001][state=COMMAND_UPLOADED] command uploaded ...
[PAYLOAD][task=target_001][state=COMMAND_REACHED] command reached ...
[PAYLOAD][task=target_001][state=PWM_CONFIRMED] channel 7 PWM confirmed ...
```

Their meanings are deliberately limited:

- `command uploaded`: the mission reported by MAVROS contains exactly one
  matching `MAV_CMD_DO_SET_SERVO` item.
- `command reached`: MAVROS reports that item reached, or mission sequence
  telemetry shows that execution passed it.
- `PWM confirmed`: `/mavros/rc/out` reported channel 7 near 1900 microseconds
  for the configured number of consecutive samples.

None of these messages proves that a physical servo moved or that the release
mechanism opened. Physical success requires independent hardware/mechanical
feedback.

Configuration is in `src/uav_payload/config/payload_monitor.yaml`. The SITL
bringup launch starts the monitor automatically. Its diagnostics are published
on `/payload/monitor/status`; inspect them with:

```bash
ros2 topic echo /payload/monitor/status
ros2 topic echo /mavros/rc/out
```

## Unified application log prefixes

Production nodes use a shared structured format:

```text
[MODULE][task=target_001][state=STATE] event key=value
```

The fixed modules are `[MISSION]`, `[PLAN]`, `[FCU]`, `[PAYLOAD]`,
`[RESTORE]`, `[SAFETY]` and `[STATUS]`. The mission manager publishes the
current task ID on `/mission/active_task_id` with transient-local QoS; the FCU
interface and payload monitor subscribe to that read-only context topic. ROS 2
launch process prefixes remain unchanged.
