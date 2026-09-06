# 测试脚本：摄像头录像 300 秒 + 飞控经纬高 sidecar

本文说明 `scripts/test_camera_geo_sidecar.sh` 做什么，以及**从飞机上电到能跑这个脚本**每一步输入什么、看到什么算正常。

脚本路径：

```text
/home/nx163/youth-vision-runtime/scripts/test_camera_geo_sidecar.sh
```

JSONL 字段和 `frame_index` 对应规则与 `GEO_FRAME_SIDECAR.md` 相同。本脚本录的是**相机原流**（1440×1080、约 60 fps、NVIDIA H.264），不是识别 overlay JPEG。

---

## 1. 这个脚本做了什么

它按 `scripts/start_camera_recording_300s.sh` 同一套方式占用海康相机：

1. 若已有 `record_camera_300s.py` 在录，直接退出。
2. 调用 `stop_youth_pipeline.sh`，停掉识别 / 网页 / 桥接（相机独占，不能和推理同时开）。
3. 在**同一个输出目录**里准备：
   - `camera_<时间>_1440x1080_300s.mp4`
   - `camera_<时间>_1440x1080_300s.jsonl`
4. source ROS 2 Humble 和 `uav_ros2_project` 的 install，以便订阅 MAVROS。
5. 后台启动 `scripts/record_camera_300s.py`：`--duration 300`、`--fps 60`、`--sidecar` 指向上面的 JSONL。
6. 脚本**马上返回**（和 300 秒录像脚本一样），不等待录完。相机进程自己在 300 秒后结束并封口 MP4。

取流循环里每成功写入一帧画面，立刻追加一行经纬高 JSONL（`frame_index` 从 0 连续加一）。不烧字幕，不发 `/vision/target_command`，不做坐标解算。

提前停：

```bash
/home/nx163/youth-vision-runtime/scripts/stop_camera_recording.sh
```

请用该脚本或 `SIGTERM`，不要 `kill -9`。

---

## 2. 从飞机上电到能运行：逐步操作

下面按本机（用户 `nx163`，工作区 `/home/nx163/...`，ROS_DOMAIN_ID 默认 0）来写。需要两件事同时成立：**飞控位置话题在发**，**海康相机空闲**。

### 步骤 A. 上电、等机载电脑起来

1. 飞机供电，飞控（ArduPilot）启动。
2. Jetson 启动。本机网口上应能看到飞控网段，例如：
   - Jetson：`192.168.144.100`（或 `.25`）
   - 飞控默认地址（bringup 里）：`192.168.144.14:15001`
3. 用 SSH 登录（在地面电脑上）：

```bash
ssh nx163@192.168.144.100
```

或本机已登录图形界面时，直接开终端。

正常：能登录，`hostname` 为 `nx163-desktop`。

### 步骤 B. 启动 ROS 2 飞控链路（必须先于本脚本）

本脚本**不会**自己启动 MAVROS。没有这一步，视频仍会录，但 JSONL 里经纬高可能全是 `null`。

本机 **没有** 启用 `uav-bringup.service`（`systemctl` 显示 inactive），仓库里的 `scripts/start_uav.sh` 还写着旧路径 `/home/hkustgz26/...`，**不要用那个脚本**。在 Jetson 上手动启动：

打开一个终端，一直留着，不要关：

```bash
source /opt/ros/humble/setup.bash
source /home/nx163/uav_ros2_project/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1
cd /home/nx163/uav_ros2_project
ros2 launch uav_bringup real_bringup.launch.py
```

刚起来时应看到类似：

```text
========== REAL FLIGHT PROFILE ==========
MAVROS FCU URL: udp://0.0.0.0:15001@192.168.144.14:15001
...
dry_run: true
allow_mission_upload: false
allow_mode_change: false
enable_real_payload_release: false
=========================================
```

随后 MAVROS、`fcu_interface_mavros_node`、`mission_manager_node`、`payload_monitor_node` 会陆续起来。飞控连上后，日志里会出现心跳 / GPS / 位置相关打印（具体字样随 MAVROS 版本略有不同）。

默认 `dry_run:=true`，**不会**改飞控模式、不会上传航线。只是为了让位置话题活着。

另开一个终端检查话题（同样要 source）：

```bash
source /opt/ros/humble/setup.bash
source /home/nx163/uav_ros2_project/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 node list
ros2 topic echo /mavros/global_position/global --once
ros2 topic echo /mavros/global_position/rel_alt --once
```

正常示例：

```text
# node list 中应有
/mavros
/fcu_interface_mavros_node
/mission_manager_node

# global --once 应打出 latitude / longitude / altitude，且不是 0,0
# rel_alt --once 应打出 data: 一个以米为单位的浮点数（地面接近 0）
```

若 `echo` 一直卡住：飞控没连上，或 `ROS_DOMAIN_ID` 不一致，或没 source workspace。先不要跑录像脚本。

### 步骤 C. 确认相机空闲

海康相机插在 Jetson USB3 上。识别 pipeline 和本录像**不能同时开**。脚本启动时会自己停识别。

不要先开：

```bash
/home/nx163/youth-vision-runtime/scripts/start_camera_recording_300s.sh
/home/nx163/youth-vision-runtime/scripts/start_youth_pipeline.sh
```

若不确定，可先：

```bash
/home/nx163/youth-vision-runtime/scripts/stop_youth_pipeline.sh
/home/nx163/youth-vision-runtime/scripts/stop_camera_recording.sh
```

### 步骤 D. 启动本脚本

再开一个终端：

```bash
# 默认录满 300 秒
/home/nx163/youth-vision-runtime/scripts/test_camera_geo_sidecar.sh

# 仅联调用，例如 15 秒（1～300）
/home/nx163/youth-vision-runtime/scripts/test_camera_geo_sidecar.sh --duration 15
```

启动成功时，**当前终端立刻**打出类似：

```text
pipeline stopped
recording started
pid=12345
output_dir=/home/nx163/youth-vision-runtime/logs/geo_sidecar_tests/run_20260817_231500
video=/home/nx163/youth-vision-runtime/logs/geo_sidecar_tests/run_20260817_231500/camera_20260817_231500_1440x1080_300s.mp4
sidecar=/home/nx163/youth-vision-runtime/logs/geo_sidecar_tests/run_20260817_231500/camera_20260817_231500_1440x1080_300s.jsonl
log=/home/nx163/youth-vision-runtime/logs/geo_sidecar_tests/run_20260817_231500/camera_geo_record.log
automatic_stop_seconds=300
stop_early=/home/nx163/youth-vision-runtime/scripts/stop_camera_recording.sh
follow_log=tail -f .../camera_geo_record.log
```

跟着看录像进程日志：

```bash
tail -f /home/nx163/youth-vision-runtime/logs/geo_sidecar_tests/run_时间/camera_geo_record.log
```

正常应先有：

```text
SIDECAR_ENABLED path=...jsonl
RECORDING_STARTED output=...mp4 resolution=1440x1080 camera_fps=... video_fps=60.00 encoder=nvv4l2h264enc max_seconds=300.0 sidecar=...jsonl
```

之后大约每 10 秒一行：

```text
RECORDING_PROGRESS seconds=10.0 frames=... capture_fps=...
```

到点后：

```text
RECORDING_COMPLETE output=...mp4 metadata=...json seconds=300.xx frames=... sidecar=...jsonl sidecar_rows=...
```

`sidecar_rows` 应等于 `frames`（都是取流循环里 `write` 的次数）。

### 步骤 E. 录完后看文件

同一目录里至少有：

| 文件 | 含义 |
|---|---|
| `camera_*_1440x1080_300s.mp4` | H.264 相机录像 |
| `camera_*_1440x1080_300s.jsonl` | 与取流写入次数 1:1 的经纬高 |
| `camera_*_1440x1080_300s.json` | 录像元数据（含 `frame_count`、`sidecar_rows`） |
| `camera_geo_record.log` | 上述 RECORDING_* 日志 |

读法与桥接 sidecar 相同：JSONL 第 N 行的 `frame_index` 对应这次录像里写入的第 N 帧（从 0 起）。H.264 有损，用播放器解码出来的帧数有时会和 `sidecar_rows` 差几帧；以 JSONL 行数和元数据里的 `frame_count` 为准。

抽查经纬高是否写上了：

```bash
head -n 1 /home/nx163/youth-vision-runtime/logs/geo_sidecar_tests/run_时间/camera_*.jsonl
```

应看到非 `null` 的 `latitude` / `longitude`。若全是 `null`，回到步骤 B 查 MAVROS。

---

## 3. 启动失败时常见输出

| 现象 | 原因 |
|---|---|
| `ROS setup not found` | `/opt/ros/humble/setup.bash` 不存在 |
| `recording already running` | 上一次 300 秒录像没停 |
| 启动后 2 秒 `RECORDING_FAILED no USB3Vision camera` | 相机未接或被识别进程占用 |
| `NVIDIA H.264 writer failed to open` | GStreamer / nvv4l2h264enc 不可用 |
| `another camera recording is already running` | `/tmp/youth_camera_recording.lock` 被占用 |
| 有 MP4 但 JSONL 经纬高全 `null` | real_bringup / MAVROS 没起，或 DOMAIN_ID 不是 0 |
| 进程马上退出、log 里 `rclpy` / `sensor_msgs` | 启动脚本没 source 到 ROS（本脚本会 source；不要绕过它直接跑 py） |

---

## 4. 和「只录 300 秒、不写经纬高」的关系

不需要 JSONL 时仍用原来的：

```bash
/home/nx163/youth-vision-runtime/scripts/start_camera_recording_300s.sh
```

默认写到 `/home/nx163/camera_recordings/`，不订飞控。本测试脚本在同一套 Python 录像程序上加了 `--sidecar`，并把 mp4/jsonl 放到 `logs/geo_sidecar_tests/run_*`。
