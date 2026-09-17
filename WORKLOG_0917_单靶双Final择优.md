# 0917 单靶双 Final 择优

日期：2026-09-17

## 1. 改动原因

旧单靶 bridge 收到第一个合法 `finalized` 坐标后立即锁定，后续质量更高的 final
无法替换。当天两次成功飞行中：

| 飞行 | 第一个 final | 第二个 final | 间隔 |
| --- | ---: | ---: | ---: |
| `real_20260917_114941_033` | 36 帧爬升包 | 55 帧平飞包 | 约 21.15 s |
| `real_20260917_115310_257` | 46 帧爬升包 | 50 帧平飞包 | 约 20.22 s |

两个任务实际发送的都是更早的爬升包。新逻辑通过比较前两个合法 final，降低起飞
爬升阶段较弱坐标抢先锁定的概率。

## 2. 当前单靶规则

`start_single_target_fusion.sh` 为 bridge 传入：

```text
target_selection_candidate_count=2
candidate_collection_timeout_sec=25.0
```

执行顺序：

1. 收到第一个合法 final 后只缓存，不发布飞控坐标；
2. 收到第二个不同 `target_id` 的合法 final 后立即比较；
3. 按以下优先级选择一个坐标：

```text
observation_count 更大
→ horizontal_radius_95_m 更小
→ confidence 更高
→ 完全相同时选择后到包
```

4. 选择完成后才进入原有 `waiting_for_standby -> publish -> manager ack` 流程；
5. 如果第一个 final 后 `25 s` 仍没有第二个，则发布当前唯一最佳包，避免无限等待；
6. 重复发布的相同 `target_id` 不会被当作第二个候选；
7. 一旦选定，后续 final 仍不再替换，bridge 只重试同一个坐标。

这里的“两个”指按时间先后到达的前两个合法 final，再从两者中选强者，并不是等待
整个航程结束后从所有包里选前两名。

## 3. 与赛事链路隔离

bridge 的默认候选数仍为 `1`，只有单靶启动脚本显式设置为 `2`。因此赛事选择器
发布到 `/vision/competition_selected_target` 的唯一最终目标仍会立即进入 bridge，
三标靶收集、10 米压制、1 秒稳定及数字/图案规则均未改变。

下游接口保持不变：

- bridge 输出：`/vision/target_command`；
- MissionManager 航点门槛：`current_mission_seq < 5`；
- 飞控调用：`goto -> push验证 -> set_current确认`；
- 舵机和航线参数不变。

## 4. 验证

NX163 已完成：

1. `uav_vision_bridge` 定向 `colcon build` 成功；
2. 安装目录确认包含 `target_selection_candidate_count` 新逻辑；
3. bridge 模块测试 `10 tests, 0 failures, 1 skipped`，其中 7 个桥接功能测试全部通过；
4. MissionManager 安全测试 `28/28` 通过；
5. 现有侦察到航线规划的安全全链路 dry-run 通过；
6. `36` 帧弱包后接 `55` 帧强包时选择第二包；
7. 第二包更弱时保留第一包；
8. 同帧数时选择 r95 更小的包；
9. 相同 `target_id` 重复消息不会凑满两个候选；
10. 25 秒超时能够使用唯一候选继续链路。

## 5. 部署与回退

NX163 更新文件：

- `/home/nx163/uav_ros2_project/scripts/start_single_target_fusion.sh`
- `/home/nx163/uav_ros2_project/src/uav_vision_bridge/uav_vision_bridge/vision_target_bridge_node.py`
- `/home/nx163/uav_ros2_project/src/uav_vision_bridge/test/test_target_retry.py`
- 重新生成的 `/home/nx163/uav_ros2_project/install/uav_vision_bridge/`

部署前备份：

```text
/home/nx163/uav_ros2_project/backups/20260917_single_target_two_final_selection/
```

## 6. 尚未覆盖

本轮未执行真实飞行。`25 s` 来自当天两个约 20 至 21 秒的实测包间隔；下一次实飞
需要重点确认第二个平飞 final 能在航点 5 前完成选择和航线改写。若航线速度或航点
距离改变，应重新评估超时值或改成明确的航点触发选择。
