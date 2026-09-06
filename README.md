# CUADC UAV Recon Fusion

固定翼无人机比赛侦察、视觉识别、坐标解算及飞控接口代码的统一私有仓库。

当前基线来自 `nx163-desktop`，导出日期为 `2026-09-07`。仓库保存当前源码、配置、航点、测试代码、systemd 服务和部署说明；模型二进制、TensorRT engine、录像、日志与识别结果不直接进入 Git。

> **安全边界**：`flight-ros2` 中包含任务上传、模式切换、航点改写和载荷接口代码。未经拆桨、无载荷或批准的 SITL/DRY_RUN 验证，不得在真实飞机上直接运行飞控入口。

## 目录

| 目录 | 内容 |
| --- | --- |
| `vision-runtime/` | 海康相机采集、Pose 三点检测、数字/图案分类、无框录像、比赛选择器和网页程序 |
| `flight-ros2/` | ROS 2 飞控接口、侦察坐标解算、任务管理、视觉桥接、航点及测试代码 |
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

开机服务：

```text
/etc/systemd/system/youth-vision.service
```

服务默认启动比赛 `image` 模式，并保存最长 `600 s`、`1440x1080`、`60 FPS` 的无模型框原始视频。

## 当前坐标解算基线

- 相机画面正上方指向机头。
- 光轴向下并朝机头方向偏转 `20 deg`。
- `camera_offset_flu_m: [0.418, 0.0, -0.09]`。
- 相机内参来自 11 x 8 棋盘格、6 mm 镜头、1440 x 1080 标定。
- 遥测轮询 `120 Hz`，结果写入 `20 Hz`。
- RTK 固定解最低 `fix_type >= 6`。
- 单目标至少 5 次观测、观测跨度至少 0.20 s。
- 最终坐标 `R95 <= 6 m`，并使用画面中心距离加权。

详细参数以 `flight-ros2/src/uav_recon/config/recon.yaml` 为准。

## 模型说明

TensorRT engine 与 Jetson、JetPack、TensorRT 和 CUDA 环境相关，不能假设 NX163 与 NX164 可直接互换。仓库只跟踪当前实际模型名称和哈希，详见 `models/README.md`。

## 双机同步原则

NX163 与 NX164 使用同一 `main` 基线，开发通过短期功能分支完成，不维护两套长期分叉源码。用户名路径、相机序列号和设备生成的 engine 必须在部署前单独核验。具体流程见 `docs/DEVICE_SYNC_GUIDE.md`。

## 本次导出状态

当前源码和配置已完成大文件、常见凭据、私钥及跨设备 `/home/nx164` 残留扫描。导出后检查时 NX163 未枚举到海康相机，开机服务因此重试；日志为 `MVS camera ... not found; available serials: []`，这是硬件未连接状态，不是 600 秒参数回退。

