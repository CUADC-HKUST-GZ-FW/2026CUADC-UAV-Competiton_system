# 视觉录像帧与飞机经纬高 sidecar

本文说明桥接节点如何把「一帧画面」和「这一帧时刻的飞机经纬高」做成代码可读的一对文件。经纬高**不烧进画面**，而是写在与 MP4 同名的 JSONL 里。

视觉识别主程序 `youth_vision_runner` 本身不录视频，只覆盖写 `overlays/latest.jpg`。按帧保存画面和位置，是 `geo_bridge_node` 在做。

---

## 1. 一对文件，而不是拆帧存图

每次开启录像，桥接在同一目录写下两个同名不同后缀的文件：

```text
<stem>.mp4      视觉 overlay 画面（MPEG-4 Part 2 / mp4v，有损）
<stem>.jsonl    与上面逐帧一一对应的飞机经纬高（UTF-8 文本，float64）
```

默认目录：

```text
/home/nx163/youth-vision-runtime/logs/geo_recordings/
```

飞行/一体化启动时的典型文件名：

```text
/home/nx163/youth-vision-runtime/logs/geo_recordings/geo_overlay_YYYYMMDD_HHMMSS.mp4
/home/nx163/youth-vision-runtime/logs/geo_recordings/geo_overlay_YYYYMMDD_HHMMSS.jsonl
```

测试脚本会改到单独目录，见 `TEST_CAMERA_GEO_SIDECAR.md`。那条测试走的是海康原流 60 fps H.264（`record_camera_300s.py`），JSONL 字段与 `frame_index` 规则与本文相同；对应的是相机写入的每一帧，不是 overlay JPEG。

命名规则：JSONL 路径 = MP4 去掉 `.mp4` 再加 `.jsonl`。不要改其中一个的文件名而不改另一个。

---

## 2. 和视频帧怎么对应

主键是 **`frame_index`**，从 `0` 起，连续加一。

| JSONL | MP4 |
|---|---|
| 第 1 行，`frame_index: 0` | `cv2.VideoCapture.read()` 的第 1 帧 |
| 第 2 行，`frame_index: 1` | 第 2 帧 |
| 第 N 行，`frame_index: N-1` | 第 N 帧 |

保证 1:1 的写法：只有 `VideoWriter.write()` 成功之后，才 `flush` 一行 JSONL。失败则这一帧两边都不算。结束时应有：

```text
sidecar 行数 == 用顺序 read 数出来的视频帧数 == 最后一行的 frame_index + 1
```

不要用墙钟或 MP4 容器里的播放时间去对帧。容器标称 `record_fps`（默认 15）只是写入时的 fps 字段；真正写入是 `latest.jpg` 一更新才写一帧，间隔不均匀。OpenCV 按恒定 fps 封装，**播放时间 ≠ 拍摄时刻**。`timestamp_unix_ms` 只是辅助，解算时仍以 `frame_index` 为准。

这些帧也**不是**相机 60 fps 原流。识别进程大约每 `overlay_every`（当前为 3）个相机帧才更新一次 `latest.jpg`，桥接录的是这些预览图。

---

## 3. 什么时候写、写什么

桥接每 50 ms（`overlay_period_sec`）看一次 `overlays/latest.jpg` 的修改时间。变了才：

1. `cv2.imread` 读入 BGR 图
2. 取当前缓冲里最新的飞机位置（MAVROS `/mavros/global_position/global` 与 `/mavros/global_position/rel_alt`）
3. 写入 MP4 一帧
4. 立刻追加 JSONL 一行并 flush

默认 `draw_hud: false`，MP4 里没有 LAT/LON 字幕。需要人眼看时再打开该参数；代码请读 JSONL，不要 OCR。

JSONL 一行一个 JSON 对象，字段如下：

| 字段 | 类型 | 含义 |
|---|---|---|
| `frame_index` | int | 与 MP4 顺序帧号相同，从 0 开始 |
| `timestamp_unix_ms` | int | 写入这一帧时的 Unix 毫秒（桥接本机墙钟） |
| `latitude` | float 或 null | 飞机纬度，度 |
| `longitude` | float 或 null | 飞机经度，度 |
| `relative_alt_m` | float 或 null | 相对起飞点高度，米 |
| `amsl_m` | float 或 null | 海拔 AMSL，米（来自 `NavSatFix.altitude`） |
| `telemetry_age_sec` | float 或 null | 这组遥测相对写入时刻有多旧，秒 |
| `source` | string | 最后更新遥测的话题：`global` / `rel_alt` / `none` |

还没有收到飞控位置时，经纬高为 `null`。JSON 浮点按 IEEE float64 写出，精度高于 1e-7～1e-8 度的需求。

示例：

```json
{"frame_index": 0, "timestamp_unix_ms": 1771460000123, "latitude": 22.8847501, "longitude": 113.4961007, "relative_alt_m": 32.51, "amsl_m": 45.12, "telemetry_age_sec": 0.041, "source": "global"}
```

---

## 4. 怎么读（给后续坐标解算用）

顺序同时读两个文件。不要按 `CAP_PROP_POS_FRAMES` 随机 seek，`mp4v` 上这个属性经常不准。

```python
import json
import cv2

stem = "/home/nx163/youth-vision-runtime/logs/geo_recordings/geo_overlay_20260817_221900"
cap = cv2.VideoCapture(stem + ".mp4")
with open(stem + ".jsonl", encoding="utf-8") as sidecar:
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        geo = json.loads(sidecar.readline())
        assert geo["frame_index"] is not None
        lat = geo["latitude"]
        lon = geo["longitude"]
        rel_alt = geo["relative_alt_m"]
        amsl = geo["amsl_m"]
        # frame 是这一时刻的画面；lat/lon/alt 是同一行里的飞机位置
```

读完后 sidecar 不应再有剩余非空行。若行数和 `cap.read` 次数不一致，说明录像被 `kill -9` 打断，或文件被只改了一半。结束桥接请用 `SIGTERM` / `stop_youth_pipeline.sh`，等进程自己关掉 `VideoWriter`。

---

## 5. 不要和另外两个 JSONL 搞混

| 文件 | 谁写 | 一行对应什么 |
|---|---|---|
| `logs/recognition_events_latest.jsonl` | `youth_vision_runner` | **一次检测事件**，无经纬高 |
| `logs/geo_bridge/fused_*.jsonl` | 桥接 | **一次检测** + 当时飞机位置，**不是**每个视频帧 |
| **`geo_overlay_*.jsonl`（本文件）** | 桥接，与 MP4 同时写 | **每一个被录进 MP4 的 overlay 帧** |

后续按视频做解算，只用 `geo_overlay_*.mp4` + `geo_overlay_*.jsonl`。

---

## 6. 怎么启动才会写出这对文件

一体化（识别 + 网页 + 桥接 + 录像）：

```bash
/home/nx163/youth-vision-runtime/scripts/start_youth_geo_pipeline.sh digit
```

只起桥接（识别已经在跑）：

```bash
/home/nx163/youth-vision-runtime/geo_bridge/scripts/run_geo_bridge.sh
```

`record_video: false` 时既不写 MP4 也不写 sidecar。配置在 `geo_bridge/config/geo_bridge.yaml`。

停进程：

```bash
/home/nx163/youth-vision-runtime/scripts/stop_youth_pipeline.sh
```
