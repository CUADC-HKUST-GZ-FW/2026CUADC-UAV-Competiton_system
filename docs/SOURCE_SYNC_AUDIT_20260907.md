# NX163 源码同步审计（2026-09-07）

## 审计范围

- NX163 视觉运行目录：`/home/nx163/youth-vision-runtime`
- NX163 飞行 ROS 2 工作区：`/home/nx163/uav_ros2_project`
- NX163 开机服务：`/etc/systemd/system/youth-vision.service`
- Git 仓库目录：`youth-vision-runtime/`、`uav_ros2_project/`、`deploy/systemd/`

## 审计结果

对 Git 已跟踪的 200 个设备部署文件逐个计算 SHA-256，并与 NX163 当前文件比较。除三个中文航点文件在 Windows 和 Linux 终端中的显示编码不同外，路径对应文件的哈希完全一致，实际内容无差异。

NX163 另外发现 8 个未纳入统一仓库的候选文件：

| 类型 | 文件 | 处理 |
|---|---|---|
| 运行日志 | `youth-vision-runtime/0906.log` | 不提交 |
| 配置备份 | 两个 `youth_pipeline.yaml.before_*` | 不提交 |
| 历史重复部署 | `youth-vision-runtime/deploy_0721/` 内两个文件 | 当前仓库已有对应历史源码，不重复提交 |
| 原飞行仓库元数据 | `uav_ros2_project/.gitignore`、`.gitattributes` | 由统一仓库根配置取代 |
| 遥测残片 | `uav_ros2_project/mav.tlog.raw` | 不提交 |

没有发现只存在于 NX163、但未回收到统一 Git 仓库的新有效源码或配置。

开发机中的 `jetson_fixedwing_deployment_20260810/payload/flight/uav_ros2_project` 是较早的部署副本：关键文件与 NX163 当前版本不同，并且缺少单标靶、四标靶启动脚本。它没有反向覆盖统一仓库。当前权威版本明确以 NX163 实际运行目录和本仓库 `uav_ros2_project/` 为准。开发机根目录的《比赛识别侦察全链路最新设置说明》与仓库 `docs/` 副本哈希一致。

## 关键功能核对

- 比赛四标靶图案/数字双模式启动脚本已同步；
- 单标靶全链路启动脚本已同步；
- `empty` 在地理解算前过滤，不会写入 target；
- 相机无框原始录像上限为 `600 s`；
- systemd 开机服务文件与仓库一致；
- 当前三项 TensorRT engine 的名称、大小和 SHA-256 与 `models/NX163_MODEL_ARTIFACTS.sha256` 一致；
- 坐标参数为相机向前倾斜 `20 deg`、`camera_offset_flu_m: [0.418, 0.0, -0.09]`、确认结果 `R95 <= 6 m`。

## 本次新增

- 可复用的设备部署文件哈希审计工具：`deploy/audit_tracked_files.py`；
- 单标靶隔离全链路 dry-run 测试；
- 单标靶安全复测结果说明。

模型 engine、训练权重、录像、识别结果、日志和密钥继续只通过制品/诊断目录管理，不直接提交 Git。
