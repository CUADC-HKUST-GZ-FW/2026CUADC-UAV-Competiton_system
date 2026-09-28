# 投弹点解算模式

所有含投弹的启动方式共用 `--release-point-mode` 参数：

- `fixed`：默认值。沿用原有固定偏移 R 点和 A→R→投放→D 航线，不在飞行中改写航线。
- `dynamic`：使用 A→U→R_safe→投放→D 初始航线；飞机越过虚拟 B 点时冻结一段飞行状态，计算新 R 点，并在到达 R 前完成第二次完整航线上传、回读和核验。计算或核验失败时保留 R_safe。
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
