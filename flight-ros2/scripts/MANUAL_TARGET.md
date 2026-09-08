# 手动坐标启动

```bash
/home/nx163/uav_ros2_project/scripts/start_manual_target.sh 22.8829116 113.4880722 0
```

三个参数依次为纬度、经度、航向角。脚本启动 bringup，同时通过同目录的
`manual_target_sender.py` 把坐标保存在进程内存中。无需重新编译 ROS 包，
但更新时必须同时复制这两个脚本。

`[TARGET][PENDING]` 表示尚未发布，后面列出等待原因。等待没有 30 秒超时。
发送器保持同一个 ROS 发布者，只有新收到的 `/mission/safety_status`
明确报告 `STANDBY`、`ready_for_attack=true`、没有阻塞项、尚未执行且没有
有效目标，同时目标话题包含已发现的 Mission Manager 订阅者且匹配数量
覆盖已发现的订阅者时，才发送一次。录包节点单独存在不能触发发送。

`[TARGET][SENT]` 表示消息已发布，不代表任务已接受。Mission Manager
仍在接收时执行完整安全检查；若状态在两者之间变化，消息仍可能被拒绝。
发送器不会自动重发，需查看任务节点的 `target accepted` / `target rejected`
记录确认。接受后会自动开始任务，等待期间没有额外的确认提示。

Ctrl+C 或终止启动脚本会取消待发坐标并停止本次启动的进程；坐标不写盘，
重启不会恢复。启动进程退出时也会取消发送器。

纯逻辑测试：`python3 scripts/test_manual_target_sender.py`。
仅使用虚拟 ROS 节点的集成测试（无需启动 bringup 或连接飞控）：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 MANUAL_TARGET_ROS_TEST=1 \
  python3 scripts/test_manual_target_sender.py
```
