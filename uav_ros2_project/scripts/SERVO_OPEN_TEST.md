# 独立舵机打开位置检测

无需启动完整链路、无侦察链路或其他模式。确认其他模式及 MAVROS 均已停止，
然后直接运行：

```bash
chmod 755 /home/nx163/uav_ros2_project/scripts/start_servo_open_test.sh
/home/nx163/uav_ros2_project/scripts/start_servo_open_test.sh 7
```

该命令只启动一个 MAVROS 和只读日志节点，不启动 MissionManager、飞控任务接口、
侦察、视觉桥接或航线上传节点。记录节点只订阅 `/mavros/rc/out` 和
`/mavros/global_position/global`，不会上传任务、切换模式或写入舵机。检测程序
默认要求第 7 通道先连续三帧处于关闭值 `1350 ± 20 us`，
随后连续三帧处于释放值 `1900 ± 20 us`，才确认一次打开。确认后必须再次连续
三帧回到关闭值，才会检测下一次打开。

每次确认会同时保存第一帧进入释放范围的位置和第三帧确认时的位置。结果目录：

```text
~/uav_flight_logs/servo_open_test/servo_open_YYYYMMDD_HHMMSS_mmm/
├── run_manifest.json
└── servo_open_events.csv
```

也可以绕过脚本直接启动，并调整飞控连接或检测参数：

```bash
ros2 launch uav_payload servo_open_test.launch.py \
  fcu_url:=udp://0.0.0.0:15001@192.168.144.14:15001 \
  servo_channel:=7 release_pwm:=1900 safe_pwm:=1350
```

为避免两个 MAVROS 同时连接飞控，启动脚本检测到 `/mavros` 已存在时会拒绝启动。
