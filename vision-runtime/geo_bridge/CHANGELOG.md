# youth-vision-runtime 地理桥接节点修改记录

日期：2026-08-17（sidecar）  
范围：只改 `geo_bridge/`、测试脚本和两份说明文档。

录像改为「MP4 + 同名 JSONL」。每一帧经纬高以 float64 写在 sidecar 里，默认不再烧字幕。约定见仓库根目录 `GEO_FRAME_SIDECAR.md`。机上联试脚本见 `TEST_CAMERA_GEO_SIDECAR.md`。

---

日期：2026-08-17  
范围：只改 `geo_bridge/`，**没有改** `youth_vision_runner` C++，也**没有改** `uav_ros2_project`。

桥接节点不再做像素→地面坐标解算。只订阅飞机实时经纬高，把最近一条遥测打到视觉 overlay 的每一帧上。与任务系统的衔接接口保留：继续读识别 JSONL，可选发布 `/vision/target_command`（默认关；本节点不再从像素算出目标 WGS84）。

订阅：

| 话题 | 类型 | 使用字段 |
|---|---|---|
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | 相对起飞点高度，米 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `latitude` `longitude` `altitude`(AMSL) |

已取消：`/mavros/global_position/compass_hdg`、`/mavros/imu/data`，以及节点内的针孔投影 / `dE dN R`。`projection.py` 仍留在目录里，供以后单独使用，直播路径不再调用。

画面默认不叠 LAT/LON 字幕（`draw_hud: false`）。识别框仍在 runner 写出的 `latest.jpg` 上。经纬高给代码读 sidecar。

---

日期：2026-08-16  
范围：只新增 `geo_bridge/` 和三条启动脚本改动，**没有改** `youth_vision_runner` C++，也**没有改** `uav_ros2_project` 的任务/飞控节点。

---

## 1. 要解决的问题

视觉流水线已经能稳定给出目标像素、编号和置信度，但：

1. 识别进程是独立 C++，不订阅飞控；
2. 任务系统 `uav_ros2_project` 只收 `/vision/target_command` 的经纬度和航向；
3. 投影需要的是拍照瞬间的真实相对高度，不能用航线参数里的 35 m。

本次补上中间层：订阅 MAVROS 高度，读识别 JSONL，把**时间上最近的一条高度**打到当前视频帧上，并算出目标相对飞机的东北天偏移。试飞时可把带这些数字的画面录成 MP4。

---

## 2. 方案选择

没有给 C++ runner 加 ROS，也没有按 `Mavlink_subscription_guide.md` 再开一条 pymavlink 串口。原因：

- `uav_ros2_project` 的 `real_bringup` 已经用 MAVROS 连着飞控。再连一次会抢报文。
- MAVLink ID 33 `GLOBAL_POSITION_INT.relative_alt` 已被 MAVROS 转成 ROS 话题 `/mavros/global_position/rel_alt`，单位已经是**米**。
- 识别结果已经按帧写在 `logs/recognition_events_latest.jsonl`，画面已经写在 `overlays/latest.jpg`。桥接节点跟文件即可，不必改推理循环。

因此采用 **旁路桥接**：

```text
youth_vision_runner          MAVROS (飞控已有)
   JSONL + latest.jpg           /rel_alt /global /hdg /imu
              \                    /
               \                  /
                geo_bridge_node
         最近高度 + 像素射线求交
                    |
     +--------------+----------------+
     |              |                |
 latest_geo.jpg   fused JSONL    geo_overlay_*.mp4
     |                               |
     叠字预览                    试飞录像
     |
     (可选，默认关) /vision/target_command
```

---

## 3. 通讯结构

### 3.1 飞控 → 桥接（ROS 2，只订阅）

| 话题 | 类型 | 使用字段 | 对应 MAVLink |
|---|---|---|---|
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | `data`，米 | ID 33 `relative_alt / 1000` |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `latitude` `longitude` `altitude` | ID 33 `lat/lon/alt` |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | `data`，度，0=北 | ID 33 `hdg / 100` |
| `/mavros/imu/data` | `sensor_msgs/Imu` | 四元数 → 滚转/俯仰 | 姿态，不是高度 |

QoS：Best Effort + Volatile，和 MAVROS 传感器话题一致。  
本机没有 `mavros_msgs` Python 包，所以没有订 `/mavros/altitude`，高度只走 `rel_alt`。

`relative_alt` 的含义是**相对 HOME/起飞点**，不是地面站电脑坐标，也不是雷达测的真 AGL。平地可以当离地高用。

### 3.2 视觉 → 桥接（文件，不经过 ROS）

跟 `start_youth_pipeline.sh` 写好的符号链接：

```text
logs/recognition_events_latest.jsonl
overlays/latest.jpg
```

JSONL 一行一个识别事件，桥接用到的字段：

```text
timestamp_unix_ms
class_label / class_prob
roi[x,y,w,h]
keypoints[3][x,y,conf]     # 0=tip  1=base_left  2=base_right
detections[]
```

像素点优先取三个关键点的几何中心，没有关键点再用框中心。  
`RecognitionTail` 按 inode/文件大小处理日志轮转，和现有 `ln -sfn` 会话日志兼容。

### 3.3 时间对齐

MAVROS 和相机不是同一时钟。做法：

1. 每条遥测到达时记下 `time.time()`；
2. 识别事件用自带的 `timestamp_unix_ms / 1000`；
3. 在最近约 200 条遥测里找墙钟最近的一条，把那条的 `relative_alt_m` 打到这一帧。

配置项 `max_match_age_sec: 0.25`：发布目标时，高度若比图像旧/新超过 0.25 s 就丢掉。叠字和录像仍会显示，并标出 `age`。

### 3.4 桥接 → 磁盘 / 任务系统

| 输出 | 路径 | 内容 |
|---|---|---|
| 叠字图 | `overlays/latest_geo.jpg` | 原 overlay + 高度 + dE/dN/R |
| 融合日志 | `logs/geo_bridge/fused_latest.jsonl` | 识别 + 高度 + 三维偏移 |
| 试飞录像 | `logs/geo_recordings/geo_overlay_YYYYMMDD_HHMMSS.mp4` | 15 fps，mp4v |
| 目标话题 | `/vision/target_command` | **默认关闭** |

`TargetCommand` 字段仍是 `latitude` `longitude` `heading_deg`。只有投影成功、置信度 ≥ 0.60、高度不过期，才会发。未标定相机前不要打开，否则经纬度是按 60° 视场估的。

任务系统要的是目标绝对经纬度，不是 `(dE, dN, dH)`。桥接里先算东北偏移，再加上飞机当前 lat/lon。航线高度 35 m 仍然只给飞控铺航点用，不参与投影。

---

## 4. 三维距离怎么算

地面按起飞平面处理：飞机在局部 NED 原点，地面在 `z = h`，`h = relative_alt - mount_z_m`。

1. 像素 `(u,v)` 用针孔模型变成相机系射线；
2. 默认相机朝下，画面上方朝机头：`body_x=-cam_y, body_y=cam_x, body_z=cam_z`；
3. 再乘安装角和飞机航向/滚转/俯仰；
4. 射线与地面相交，得到：

```text
east_m / north_m     目标相对飞机的水平偏移
down_m               等于相机离地高度
horizontal_m         sqrt(dE^2 + dN^2)
range_m              三维斜距 sqrt(dE^2 + dN^2 + h^2)
```

没有内参时用 `horizontal_fov_deg: 60` 估 `fx/fy`，融合日志里 `reason=uncalibrated_fov_model`。标定后把 `fx fy cx cy` 填进 YAML，`reason` 变为 `ok`。高度过低或射线不朝下会失败，不写假数。

画面上每个目标一行：`dE dN H R`。`H` 是这一帧打上的真实相对高度，`R` 是三维斜距。

---

## 5. 新增和改动的文件

```text
geo_bridge/geo_bridge/telemetry.py      遥测缓存，按墙钟取最近样本
geo_bridge/geo_bridge/recognition.py    JSONL 跟踪、像素点、箭头方向
geo_bridge/geo_bridge/projection.py     针孔 + 地面平面
geo_bridge/geo_bridge/overlay.py        把高度和 dE/dN/R 画到帧上
geo_bridge/geo_bridge/recorder.py       把叠字帧写成 MP4
geo_bridge/geo_bridge/node.py           ROS 2 节点
geo_bridge/config/geo_bridge.yaml       话题、相机、录像开关
geo_bridge/scripts/run_geo_bridge.sh    单独启动桥接
geo_bridge/scripts/test_geo_overlay_record.py
geo_bridge/tests/test_projection.py
geo_bridge/CHANGELOG.md                 本文件

scripts/start_youth_geo_pipeline.sh     识别 + Web + 桥接一体化
scripts/stop_youth_pipeline.sh          增加停止桥接 / 测试录像
scripts/status_youth_pipeline.sh        增加桥接进程和日志
```

未改推理引擎、未改 MAVROS launch、未改 `TargetCommand.msg`。

---

## 6. 怎么跑

先确认飞控/MAVROS 已在（实飞用 `real_bringup`）。同一 `ROS_DOMAIN_ID`（默认 0）。

一体化（识别 + 网页 + 桥接 + 录像）：

```bash
/home/nx163/youth-vision-runtime/scripts/start_youth_geo_pipeline.sh digit
/home/nx163/youth-vision-runtime/scripts/status_youth_pipeline.sh
/home/nx163/youth-vision-runtime/scripts/stop_youth_pipeline.sh
```

不录像：

```bash
/home/nx163/youth-vision-runtime/scripts/start_youth_geo_pipeline.sh digit --no-record
```

只起桥接（识别已经在跑）：

```bash
/home/nx163/youth-vision-runtime/geo_bridge/scripts/run_geo_bridge.sh
```

地面没有飞控时，用假高度把现有 overlay 录成测试片：

```bash
# 先起识别，保证 overlays/latest.jpg 在更新
/home/nx163/youth-vision-runtime/scripts/start_youth_pipeline.sh digit

python3 /home/nx163/youth-vision-runtime/geo_bridge/scripts/test_geo_overlay_record.py \
  --mock-alt 32.5 \
  --duration-sec 20
```

输出在 `logs/geo_recordings/geo_overlay_test_*.mp4`。  
试飞请用一体化脚本，高度来自 `/mavros/global_position/rel_alt`，不要用 `--mock-alt`。

投影自检：

```bash
python3 /home/nx163/youth-vision-runtime/geo_bridge/tests/test_projection.py
```

---

## 7. 试飞时看什么

1. `ros2 topic echo /mavros/global_position/rel_alt --once`  
   地面接近 0，空中是米，不是 35000（毫米没换算）。
2. 桥接日志里 `rel_alt_n` 在增加。
3. `overlays/latest_geo.jpg` 左上角有 `REL_ALT` 和 `age`。`age` 应小于约 250 ms。
4. 停进程后 MP4 会正常收尾。不要 `kill -9`。

---

## 8. 还没做、需要标定后才打开的

1. 把实测 `fx fy cx cy` 和安装角写入 YAML。现在 60° 视场只是能跑通的估计。
2. `publish_target_command: true` 才会往任务系统发目标。未标定不要开。
3. 坡地要用真 AGL / 测距，不能只用相对起飞点高度。
4. 图像时间戳和飞控时间戳没有硬件同步，目前是最近邻。
5. 本机未安装 `mavros_msgs`，没有订 `/mavros/altitude.terrain`。
