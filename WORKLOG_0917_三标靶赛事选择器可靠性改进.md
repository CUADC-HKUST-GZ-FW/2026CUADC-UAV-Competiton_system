# 0917 三标靶赛事选择器可靠性改进

日期：2026-09-17

## 1. 改进原因

当天单标靶实飞表明，同一物理标靶可能先后产生爬升包和平飞包，两个 final
坐标会相差数米，标签也可能发生抖动。比赛选择器原先只用 `3 m` 做空间去重，
因此同一标靶的两个 final 包可能被误计为两个赛事目标，并提前凑满三个候选。

另一个风险来自 Jetson 离线时钟回退。旧比赛 session 只包含秒级时间戳；若目录名
重复，旧 `target_*/result.json` 可能重新进入本次选择，日志和原始录像也可能重名。

## 2. 新的三标靶裁决流程

选择器只接收满足以下条件的结果：

1. `status=finalized`；
2. `valid=true`；
3. `rtk_fixed=true`；
4. 标签不是 `empty`，且能转换成当前数字或图案赛事值。

合格 final 包按下列质量顺序由强到弱排列：

```text
observation_count
confidence
label_consensus
-horizontal_radius_95_m
```

随后执行最强包优先的空间压制：

```text
距离 <= 10 m：视为同一物理标靶，只保留质量更强的包
距离 > 10 m：允许成为不同物理标靶
```

该算法不是传递式聚类。每个保留下来的代表包与其他代表包都必须大于 `10 m`，
不会出现 `A-B`、`B-C` 都很近却把相距很远的 `A-C` 串成一组的情况。

空间压制后继续执行全局同标签去重，防止同一数字或图案在不同位置重复占用名额。
三个不同非空代表包的 ID、标签、帧数和坐标保持不变 `1 s` 后，选择器才锁定结果：

- 数字模式：选择三个数的中位数；
- 图案模式：选择赛事排序值最高的图案。

`empty` 包不参加坐标裁决、不占三个名额，也不会发布给飞控链路。

## 3. 审计输出

`competition_selected.json` 新增：

- `distinct_target_distance_m`：当前为 `10.0`；
- `eligible_nonempty_targets`：最终独立非空目标数量；
- `suppressed_candidates`：被压制包、原因、保留包 ID 和两者距离。

日志新增 `Competition candidate suppressed`。即使三个代表包没有变化，后来出现的
新压制包也会独立记录。旧字段 `ignored_confirmed_candidates` 暂时保留，避免现有网页
读取器因字段消失而报错。

## 4. Session 隔离

比赛 session ID 从：

```text
YYYYMMDD_HHMMSS_mode_competition_fusion
```

改为：

```text
YYYYMMDD_HHMMSS_boot-id前8位_进程单调启动ticks_mode_competition_fusion
```

即使离线系统时间回退，重启后的 boot ID 也不同；同一次开机内，进程启动 ticks
不同。启动脚本还会拒绝复用已存在的 session 目录。选择器结果、桥接日志、识别日志
和比赛原始录像均使用同一个唯一 session ID，旧任务结果不能混入本次三靶选择。

## 5. 接口保持不变

以下比赛全链路接口没有修改：

- 启动命令：`start_competition_target_fusion.sh [digit|image] [heading_deg]`；
- 发布话题：`/vision/competition_selected_target`；
- 消息类型：`uav_interfaces/msg/ReconTarget`；
- 三目标稳定时间：`1.0 s`；
- bridge 的 `auto_execute=true`；
- MissionManager 的航点 5 前接收门槛、航线 push / set_current 和舵机命令。

本次只改变“哪个 final 包能代表三个真实标靶”和“任务文件如何隔离”，不改变飞控
任务或载荷指令格式。

## 6. 验证结果

NX163 实机软件环境已完成：

1. `bash -n` 启动脚本通过；
2. Python 编译检查通过；
3. 比赛选择器 13 项回归测试全部通过；
4. bridge、MissionManager、飞控日志和载荷监控共 67 项测试全部通过；
5. ROS 选择器进程启动烟测通过；
6. 合成四包验收通过：55 帧强包和 7 米外 36 帧弱包被合并为一个目标，日志记录
   `distance_m=7.000`，剩余三个目标稳定后正确选择数字中位数 `69`；
7. 测试结束后未残留视觉、侦察、选择器、桥接或任务管理进程。

## 7. 部署与回退

NX163 已更新：

- `/home/nx163/youth-vision-runtime/scripts/competition_selector_core.py`
- `/home/nx163/youth-vision-runtime/scripts/competition_selector.py`
- `/home/nx163/youth-vision-runtime/scripts/test_competition_selector_core.py`
- `/home/nx163/uav_ros2_project/scripts/start_competition_target_fusion.sh`

部署前备份位于：

- `/home/nx163/youth-vision-runtime/backups/20260917_competition_selector_v2/`
- `/home/nx163/uav_ros2_project/backups/20260917_competition_selector_v2/`

## 8. 剩余边界

本轮没有带真实相机和飞控执行比赛航线，只完成了 Jetson 软件环境下的选择器、ROS
发布和下游模块回归。若产生四个彼此大于 `10 m`、标签也不同的合法 final 包，现有
策略仍按质量选择前三个；这是可用性与“发现额外异常就拒绝发布”之间的保留取舍，
需要在下一次三标靶实拍后根据误检率决定是否改成严格恰好三个。
