# 0917 NX164 全功能同步与验收

日期：2026-09-17

## 1. 目标

将当天已在 NX163 完成的连续帧取包、单靶双 Final 择优、三标靶赛事选择、任务隔离和
全链路日志能力同步到 NX164，并针对 NX164 的独立相机、飞控和 TensorRT 环境完成
可运行性验收。

## 2. 代码同步

NX164 仓库与本地主仓库先对齐到 `e2835b5`。部署前备份位于：

```text
/home/nx164/backups/20260917_today_sync_pre_20260917_210254
/home/nx164/deployment_backups/20260917_211044_e2835b5f990a
```

`deploy/sync_local_jetson.sh` 已增加设备专属 TensorRT 路径保护。同步代码时继续保留
NX164 的相机序列号、标定参数、外参和有效引擎路径，不再把 NX163 的 engine 路径
覆盖到 NX164。

ROS 工作区使用 `colcon build --symlink-install` 全量重建，7 个包均成功：

```text
uav_interfaces
uav_fcu_interface
uav_mission_manager
uav_payload
uav_vision_bridge
uav_recon
uav_bringup
```

## 3. NX164 硬件配置

相机探测与连续取流确认：

```text
serial: DB1225040
model: MV-CB016-10GC-C
address: 192.168.10.11
resolution: 1440x1080
```

保留的 NX164 标定与外参：

```text
fx=1832.40216703
fy=1807.64337052
cx=762.64366822
cy=490.37201509
distortion=[-0.1137587303, 0.5994750345, -0.001553195991,
            0.001922931368, -1.693903038]
camera_forward_tilt_deg=20.0
camera_left_tilt_deg=0.0
camera_offset_flu_m=[0.418, 0.0, -0.09]
```

其中 `camera_forward_tilt_deg=20.0` 对应光轴相对水平向下 70 度。

飞控通过 UDP 链路实际被 MAVROS 识别，日志读取到：

```text
ArduPlane V4.8.0-dev (e4b77e0a)
ZeroOneX9 00180038 3139510C 30303432
```

本轮只读取了心跳、遥测和 14 个原任务航点，没有向真实飞控执行任务上传。

## 4. NX164 原生 TensorRT 引擎

原配置中的 Pose 和数字 engine 文件名仍带 `nx163`，运行时会报告跨设备 plan 警告。
因此使用原始 ONNX 在 NX164 本机重新构建三个 FP16 engine。

Pose 先按旧脚本的 `builderOptimizationLevel=0` 构建，真实视频只有约 `52.8 FPS`，未
达到 60 FPS，故没有部署。随后按历史已验证参数 level 3 重建并通过验收。

当前活动 engine：

| 模型 | 文件 | SHA256 |
| --- | --- | --- |
| Pose | `arrow_pose_3pt_flightdomain_0902_branch_best_fp16_opt3_nx164.engine` | `e980a3ac1136f6a3588e37a761a55afcc9e90198cb438c92f1421b96f611ab0f` |
| 图案 | `image_real_blank_gloo_best_fp16_nx164.engine` | `5ca41fc2502d67819b996e414d157c9fdac48c6e3b12933b4cda9b1f99d8491f` |
| 数字 | `digit_cls_balanced101_0903_night_v2_best_fp16_nx164.engine` | `adee62306f42e76f25307afd5a9874d6e760577ca78eb54aca644cc29eb7e31d` |

旧配置和旧 engine 的完整回退备份：

```text
/home/nx164/backups/20260917_full_sync_engines
```

仓库中的四个 Pose 构建入口也统一改为 `builderOptimizationLevel=3`，防止以后重建时
再次生成低吞吐 engine。

## 5. 模型实测

`trtexec`：

```text
Pose level 3: 145.06 qps, GPU 6.88 ms
image classifier: 1503.78 qps, GPU 0.66 ms
digit classifier: 1487.62 qps, GPU 0.67 ms
```

真实独立视频：

| 视频 | 正确类别 | 识别事件 | 同类命中 | 运行 FPS |
| --- | --- | ---: | ---: | ---: |
| 轰炸机 | `class_id=9` | 1332 | 1332 | 76.53 |
| 战斗机 | `class_id=5` | 1028 | 1028 | 77.82 |

真实相机连续取流：

```text
frames=979
wall_fps=60.11
mean_pose=7.27 ms
TensorRT cross-device warning=0
fatal error=0
```

## 6. 算法和任务参数一致性

帧包配置：

```text
tracking_mode=pixel_packets
packet_association_gap_sec=0.10
packet_gap_timeout_sec=0.20
pixel_gate=min(200, 50 + 1300 * dt)
packet_max_observations=300
packet_min_observations=11
packet_merge_distance_m=1.5
packet_distinct_distance_m=10.0
```

单靶链路：

```text
前两个不同 target_id 的合法 Final 中择优
target_selection_candidate_count=2
candidate_collection_timeout_sec=10.0
```

赛事链路：

```text
required_nonempty_targets=3
dedup_radius_m=10.0
settle_sec=1.0
图案选择最高赛事值；数字选择中位数
```

飞控任务参数保持：

```text
insert_wp_index=5
resume_wp_index=10
d_offset_m=50.0
```

## 7. 本轮补丁

### 7.1 可选识别日志崩溃

原生 runner 在没有打开识别日志时用流对象真假判断，视频回放可触发
`failed to write recognition event log`。改为 `recognition_log.is_open()` 后，带日志和
不带日志两种入口均正常。

### 7.2 独立侦察停止不完整

`stop_recon_pipeline.sh` 原来只匹配 `recon.launch.py`，实际入口为
`recon_with_mavros.launch.py` 或 `recon_static_with_mavros.launch.py`，会把有效 PID
误判为 stale，留下 MAVROS 和侦察 ROS 进程。

已扩展可信命令匹配，并补充 `competition_selector.py`。实测启动后再停止：

```text
recon_ros process group stopped
vision process group stopped
PID files removed
UDP 15001 released
```

### 7.3 MAVROS StreamRate 响应兼容

ROS Humble 当前 `mavros_msgs/srv/StreamRate` 响应为空，不含旧版本的 `success` 字段。
代码现对空响应按成功处理，对存在 `success` 字段的旧版本继续尊重其值。NX164 实测
日志由属性异常变为：

```text
FCU attitude stream requested at 60 Hz
```

## 8. 全链路验收

功能测试：

```text
ROS 功能测试: 126 passed
赛事选择器和地理解算测试: 21 passed
合计: 147 passed
```

单靶隔离全链路：12 个连续识别帧形成一个合法 Final，经 bridge、MissionManager、
`goto` 到 A/R/D 航线规划，坐标逐层一致；`allow_mission_upload=false`，真实写入次数为
零。

比赛隔离全链路：四个 Final 中，距强包 5.005 米的弱包被压制；三个独立非空目标
稳定 1 秒后选择轰炸机，经赛事 topic、bridge、MissionManager 到 A/R/D 航线规划，
坐标逐层一致；真实写入次数为零。

真机启动和停止：

1. 单靶 SSH detached 入口启动到完整 ROS 图，正式停止后无残留；
2. 比赛 SSH detached 入口启动到完整 ROS 图，正式停止后无残留；
3. 独立侦察入口启动后由修复后的 `stop_recon_pipeline.sh` 完整关闭；
4. 比赛选择器、侦察、bridge、任务管理器、MAVROS 和视觉 runner 均无残留 PID。

## 9. 当前状态与边界

`youth-vision.service` 当前为 `disabled/inactive`，本轮没有擅自改变 NX164 的上电启动
策略。其 unit 中仍是旧的 `competition_selected.sh image`，若以后要启用开机自启，
应先明确需要单靶还是比赛入口再改 unit。

本轮没有在地面向真实飞控执行 mission push、`set_current` 或舵机释放；这些动作只在
隔离 dry-run 中验证。下一次安全联调应在卸桨/禁用载荷条件下验证真实 push 回读和
`set_current` 回读，不能把本次 dry-run 描述为真实飞控写入验收。

单靶 `10 s` 兜底是当前明确设置。已知 0917 的较强平飞包可能比起飞包晚约 20 秒，
因此它仍可能在第二包到达前提交首包；这是当前策略取舍，不是代码未同步。
