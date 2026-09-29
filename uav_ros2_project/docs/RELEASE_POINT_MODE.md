# 投弹点解算模式

所有含投弹的启动方式共用 `--release-point-mode` 参数：

- `fixed`：默认值。沿用原有固定偏移 R 点和 A→R→投放→D 航线，不在飞行中改写航线。
- `dynamic`：使用 A→U→R_safe→投放→D 初始航线；飞机越过虚拟 B 点时冻结一段飞行状态，以 MAVROS 实测相对高度、水平速度和垂直运动估计 R 点释放状态，迭代计算新 R 点，并在 R 成为当前任务项之前完成第二次完整航线上传。正常成功路径检查上传 ACK、传输数量及任务进度，不做完整回读。计算失败且尚未上传时保留 R_safe；上传报错后进行一次只读回读，区分 SAFE、DYNAMIC、INCONSISTENT 和 UNKNOWN。上传后超时或取消不能断言固定 R 仍有效，将报告 UNKNOWN 并通知任务管理器进入现有 SAFE/人工处理流程。
- `shadow`：执行相同计算并记录结果，但不上传新 R 点，用于实飞观察算法输出。

比赛多标靶全链路：

```bash
./scripts/start_competition_target_fusion.sh image 0 --release-point-mode dynamic
```

单标靶全链路：

```bash
./scripts/start_single_target_fusion.sh image 0 --release-point-mode dynamic
```

无视觉、手动目标：

```bash
./scripts/start_manual_target.sh <纬度> <经度> <航向角> --release-point-mode dynamic
```

通用启动入口：

```bash
./scripts/start_uav.sh --release-point-mode dynamic
```

省略参数时均使用 `fixed`，因此原有启动命令和原有固定 R 点航线保持不变。启动脚本仍在后台运行并将日志写入原有目录。

也可直接启动 ROS 2：

```bash
ros2 launch uav_bringup real_bringup.launch.py release_point_mode:=dynamic
```

SITL 使用同一个参数：

```bash
ros2 launch uav_bringup sitl_mavros_dynamic_abc.launch.py release_point_mode:=dynamic
```

注意：任务管理器的 SAFE 状态只阻止后续伴随计算机控制，不撤销飞控已接收的航线。`shadow` 无论是否启用 `dynamic_r_test_mode` 都禁止第二次上传；初始任务仍包含 U 点。

动态高度解算的输入、公式、保护条件和验证结果见
[`DYNAMIC_R_ACTUAL_RELATIVE_ALTITUDE_20260929.md`](DYNAMIC_R_ACTUAL_RELATIVE_ALTITUDE_20260929.md)。
