# 2026CUADC UAV Competition System

固定翼无人机比赛侦察、视觉识别、坐标解算及飞控接口代码的统一私有仓库。

当前基线来自 `nx163-desktop` 的实际运行代码，最近同步日期为 `2026-09-12`。仓库保存当前源码、配置、航点、测试代码、systemd 服务和部署说明；模型二进制、TensorRT engine、录像、日志与识别结果不直接进入 Git。

> **安全边界**：`uav_ros2_project` 中包含任务上传、模式切换、航点改写和载荷接口代码。未经拆桨、无载荷或批准的 SITL/DRY_RUN 验证，不得在真实飞机上直接运行飞控入口。

## 目录

| 目录 | 内容 |
| --- | --- |
| `youth-vision-runtime/` | 海康相机采集、Pose 三点检测、数字/图案分类、无框录像、比赛选择器和网页程序 |
| `uav_ros2_project/` | ROS 2 飞控接口、侦察坐标解算、任务管理、视觉桥接、航点及测试代码 |
| `deploy/systemd/` | NX163 当前开机自启动服务定义 |
| `models/` | 当前模型文件名、大小和 SHA-256，不包含设备相关 engine |
| `docs/` | 链路说明、设备同步方法及本次源码来源记录 |

## 当前生效入口

比赛图案侦察：

```bash
/home/nx163/youth-vision-runtime/scripts/competition_selected.sh image
```

比赛数字侦察：

```bash
/home/nx163/youth-vision-runtime/scripts/competition_selected.sh digit
```

比赛四标靶全链路，第二个参数是打击航向角：

```bash
/home/nx163/uav_ros2_project/scripts/start_competition_target_fusion.sh image 90
/home/nx163/uav_ros2_project/scripts/start_competition_target_fusion.sh digit 90
```

全链路等待三个非空确认目标；数字模式选择数值中位数，图案模式选择比赛价值最高者，然后经 bridge 向 `/vision/target_command` 发布唯一坐标。若任务管理器仍处于 `STANDBY`，bridge 会在总超时内重发，直到任务管理器接收。该入口会启动真实飞控任务组件，运行前必须停止 `youth-vision.service` 和其他视觉、MAVROS、飞控进程。

开机服务：

```text
/etc/systemd/system/youth-vision.service
```

服务默认启动比赛 `image` 模式，并保存最长 `600 s`、`1440x1080`、`60 FPS` 的无模型框原始视频。

## 当前坐标解算基线

- 相机画面正上方指向机头。
- 光轴从垂直向下朝机头偏转 `12 deg`，并朝机体左侧偏转 `4.5 deg`。
- `camera_offset_flu_m: [0.418, 0.0, -0.09]`。
- 相机内参来自 11 x 8 棋盘格、6 mm 镜头、1440 x 1080 标定。
- 清单轮询 `120 Hz`；主动请求全球位置与姿态各 `20 Hz`，按相机帧时间插值。
- RTK 固定解最低 `fix_type >= 6`。
- 单目标至少 5 次观测、观测跨度至少 0.20 s。
- 最终坐标 `R95 <= 6 m`，并使用画面中心距离加权。

详细参数以 `uav_ros2_project/src/uav_recon/config/recon.yaml` 为准。

## 模型说明

TensorRT engine 与 Jetson、JetPack、TensorRT 和 CUDA 环境相关，不能假设 NX163 与 NX164 可直接互换。仓库只跟踪当前实际模型名称和哈希，详见 `models/README.md`。

## 双机同步原则

NX163、NX164 和所有队员使用同一 `main` 基线，开发通过短期功能分支和 Pull Request 完成，不维护两套长期分叉源码。Git 工作副本与机载实际运行目录分开，同步时使用 `deploy/sync_local_jetson.sh`；用户名路径、相机序列号和设备生成的 engine 会作为设备差异保留。具体流程见 `docs/DEVICE_SYNC_GUIDE.md` 和 `CONTRIBUTING.md`。

## 本次导出状态

当前源码以 NX163 的飞控提交 `df8d476`、其后尚未提交的高频遥测对齐更新，以及 NX163 当前视觉运行目录为源重新汇总。旧的 `CUADC-HKUST-GZ-FW/uav_ros2_project` 仅作为历史来源，不再是团队主仓库。
