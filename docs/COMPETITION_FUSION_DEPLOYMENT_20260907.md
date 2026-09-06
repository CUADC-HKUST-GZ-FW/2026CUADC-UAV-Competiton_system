# NX163 四标靶比赛全链路部署记录

日期：2026-09-07

## 启动入口

```bash
sudo systemctl stop youth-vision
/home/nx163/uav_ros2_project/scripts/start_competition_target_fusion.sh image 90
```

```bash
sudo systemctl stop youth-vision
/home/nx163/uav_ros2_project/scripts/start_competition_target_fusion.sh digit 90
```

第二个参数是打击航向角，单位为度。命令会启动真实飞控任务组件，不能与开机侦察服务、其他 MAVROS 或其他飞控 bringup 同时运行。

## 数据流

```text
Pose + 分类 + RTK/姿态坐标解算
  -> /vision/recon_result
  -> 三个不同的 confirmed、RTK Fixed、非 empty 比赛目标
  -> 数字中位数 / 图案最高价值
  -> /vision/competition_selected_target
  -> vision_target_bridge_node
  -> /vision/target_command（只发布一次）
  -> MissionManager
```

单标靶脚本、ROS 消息和 MissionManager 输入 topic 均未改变。新脚本只改变编排和 bridge 的输入 topic。

## 选择保护

- `empty`、未确认、坐标无效或非 RTK Fixed 结果不参与选择。
- 比赛规则要求三个非空目标的标签互不相同；同标签重复轨迹只保留质量较高者。
- 空间去重采用组内两两距离约束，禁止 A-B-C 链式跨段合并。
- 候选集合稳定 3 秒后锁定，锁定后不再更换目标。
- bridge 只在 MissionManager 为 `STANDBY` 时转发一次。

## 验证结果

- Python 选择逻辑：7 项测试通过。
- NX163 `bash -n`：通过。
- 独立 ROS domain 接口测试：通过。
- 数字样例 `10 / 50 / 90`：输出 `50`。
- 图案样例 `机枪兵 / 坦克 / 轰炸机`：输出 `轰炸机`。
- 测试没有连接真实飞控，也没有执行航点上传或载荷动作。

部署前备份：

```text
/home/nx163/backups/competition-fullchain-pre-20260907_011319
```

## 已知安全边界

当前 MissionManager 的 SAFE/disable 路径会停止伴随计算机继续发令，但代码明确记录它不能保证取消飞控中已经上传并开始执行的任务。因此，`Ctrl+C` 不能替代遥控器接管、飞控模式切换或地面站任务取消。
