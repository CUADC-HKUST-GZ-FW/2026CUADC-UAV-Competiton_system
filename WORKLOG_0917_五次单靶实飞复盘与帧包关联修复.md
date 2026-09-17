# 0917 五次单靶实飞复盘与帧包关联修复

日期：2026-09-17

## 1. 当日结论

今天五次单标靶全链路飞行中，任务 1、4 完成航线改写，任务 2、3、5
没有改写。日志证明后者并非飞控拒绝了一个已经发布的坐标，而是视觉取包阶段没有
产生 `finalized` 坐标，因此桥接器和 MissionManager 根本没有收到目标。

直接原因是旧像素门限在 60 FPS 下实际只有约 `46.7 px`，低于当天相邻有效帧
约 `60.6 px` 的 P95 位移。同一个目标被拆成多个短包后，代码又逐包执行 11 帧
门槛，导致总计 14 帧和 22 帧的连续目标集合分别以 `8+1+2+...`、
`9+2+4+...` 的形式全部被丢弃。

本轮完成以下修复：

1. 用 `0.10 s` 时间窗和放宽后的动态像素门限进行同目标关联；
2. 同一帧的多个检测统一做一对一匹配，支持多标靶同时入镜；
3. 先完成像素轨迹重组，再执行 11 帧审判和包间坐标裁决；
4. 保留 `0.20 s` 整组静默封包和 300 帧上限；
5. 增加新包创建原因日志，能直接看到时间断开、像素越界或同帧冲突；
6. 修复旧 manifest 被新任务误读、关停脚本遗漏全链路进程和离线时钟回退问题。

## 2. 五次飞行复盘

| 运行 | 视觉结果 | 飞控结果 | 判定 |
| --- | --- | --- | --- |
| `real_20260917_114941_033` | 36 帧爬升包先定稿，后有 55 帧平飞包 | 改写成功 | 全链路可用 |
| `real_20260917_114939_717` | `8+1+2+1+1+1`，无 final | 未收到坐标 | 像素门限拆包 |
| `real_20260917_114940_969` | `1+1+1+1+2`，总计 6 帧 | 未收到坐标 | 帧数确实不足 |
| `real_20260917_115310_257` | 46 帧爬升包先定稿，后有 50 帧平飞包 | 改写成功 | 全链路可用 |
| `real_20260917_115111_871` | `9+2+4+1+1+...`，无 final | 未收到坐标 | 像素门限拆包 |

三次失败识别都发生在约 4 至 6 米的着陆下降阶段、航点 12 至 13 附近，
不属于正常 35 米侦察平飞。它们关闭包时已经越过航点 5，即使新算法生成坐标，
MissionManager 也应按安全规则拒绝迟到目标，不能绕过该门槛强行改航。

当天确实存在两个质量很高的平飞包：

- 55 帧，约 32.67 米，航点 4；
- 50 帧，约 32.47 米，航点 4；
- 两个平飞结果相距约 0.70 米。

两次成功任务传给飞控的是更早的 36/46 帧爬升包。单标靶 bridge 当前按设计锁定
第一个合法 final，后续平飞包不会替换它。这不是三次“没有改写航线”的原因，
但仍是后续提升坐标准度时需要单独决策的问题。为保证航点 5 前及时交付，本轮没有
擅自增加等待多个单靶 final 的门槛。

## 3. 旧算法为何错误拆包

旧门限为：

```text
gate(dt) = min(200, 25 + 1300 * dt)
```

`200 px` 只是上限，不是每帧实际门限。在 60 FPS 下：

```text
dt = 1 / 60 s
gate = 25 + 1300 / 60 = 46.7 px
```

当天相邻中心位移经常达到 48 至 93 px，所以连续目标会被切开。旧审判顺序是：

```text
逐包检查 >= 11 帧
  -> 不足立即丢弃
  -> 剩余包做坐标融合
  -> 1.5 m / 10 m 包间裁决
```

因此多个短包即使总帧数超过 11，也没有机会恢复成一个有效目标。

## 4. 新连续帧算法

### 4.1 同目标关联

新参数为：

```text
association_dt <= 0.10 s
gate(dt) = min(200, 50 + 1300 * dt)
group_quiet_timeout = 0.20 s
packet_max_observations = 300
```

典型门限：

| 时间间隔 | 像素门限 |
| ---: | ---: |
| 1/60 s | 71.7 px |
| 1/30 s | 93.3 px |
| 0.05 s | 115 px |
| 0.10 s | 180 px |

`0.10 s` 只决定一个检测还能否接到某条既有轨迹；`0.20 s` 决定整组何时结束。
两者分离后，明确断开的目标不会被无限续接，但同一时段的多个候选轨迹仍可一起进入
最终包间裁决。

### 4.2 同帧多目标

每一帧的所有非空 Pose 检测先集中起来，再与活动轨迹执行一对一匹配：

1. 不使用瞬时分类标签做关联，避免一两帧误分类拆包；
2. 以 `像素距离 / 当前动态门限` 从小到大匹配；
3. 一个活动包在同一帧最多接收一个检测；
4. 一个检测最多属于一个活动包；
5. 未匹配检测创建新包。

这使两个以上标靶同时出现在画面时不会因为检测遍历顺序被塞进同一个包。

### 4.3 封包、审判与坐标裁决

处理顺序固定为：

```text
非空三点 Pose
  -> 0.10 s + 动态像素门限重组轨迹
  -> 0.20 s 静默或 300 帧上限封组
  -> 每条重组轨迹至少 11 帧
  -> 包内 MAD 离群剔除和中心加权坐标融合
  -> 有效帧多数投票决定标签
  -> empty 多数包丢弃，不发布、不进入网页
  -> 包间 1.5 m / 10 m 裁决
  -> 只发布 finalized
```

包间规则保持不变：

| 坐标距离 | 动作 |
| --- | --- |
| `<= 1.5 m` | 按有效帧数合并坐标和标签票数 |
| `> 1.5 m` 且 `< 10 m` | 丢弃帧数更少的包 |
| `>= 10 m` | 保留为不同标靶 |

## 5. 新增诊断日志

新建包时输出：

```text
[RECON_PACKET] stage=packet_started
```

日志包含 `packet_id`、帧号、中心像素及以下原因之一：

- `no_active_packet`
- `association_gap_exceeded`
- `pixel_gate_exceeded`
- `same_frame`
- `one_to_one_conflict`
- `max_observations`

最终结果继续输出：

```text
[RECON_PACKET] stage=group_finalized
[RECON_PACKET] stage=coordinate_finalized
[FULLCHAIN] stage=coordinate_published
[FULLCHAIN] stage=manager_received
[FULLCHAIN] stage=goto_requested
[FULLCHAIN] stage=push_verified
[FULLCHAIN] stage=set_current_confirmed
```

结果元数据新增 `association_gap_sec`、动态门限基值和变化率，便于以后直接根据日志
复原运行参数。

## 6. 同日配套修复

### 6.1 启动前 manifest 隔离

三个视觉启动入口在新 producer 启动前删除旧的单帧交接文件；recon 节点启动时也会
记住并忽略已经存在的 manifest。历史网页结果保存在
`recon_results/sessions/*/target_*/result.json`，不依赖该文件，因此不会消失。

### 6.2 完整关停

`stop_recon_pipeline.sh` 现在依次处理：

- 活跃的 `youth-vision.service`；
- detached 单靶父进程；
- 带 boot ID 和进程启动 tick 校验的 lock owner；
- legacy web、selector、recon、vision、bridge 和 bringup pidfile；
- SIGINT、SIGTERM、必要时 SIGKILL 的分级退出。

它不再只关闭旧的独立侦察 pidfile，从而避免下一次任务与残留单靶链路并行。

### 6.3 离线时钟

增加 `uav-clock-persist.timer`，每 30 秒持久化最近的有效系统时间，并让
`youth-vision.service` 排在 `time-set.target` 之后。它能阻止无网络冷启动时严重回退
到 1970 年，但突然断电仍可能回退到最近一次持久化时间，最大约 30 秒。

### 6.4 相机标定

NX163 使用 2026-09-16 新棋盘格参数：47/73 张有效图，RMS 约 `0.094184 px`。
本轮没有改变 70 度朝前、侧偏 0 度的外参定义。

## 7. 验证结果

### 7.1 当天日志回放

使用新算法回放当天原始事件：

- 原 14 帧运行时失败集合对应的 17 个原始观察恢复为一个 17 帧包；按当天遥测
  对齐损失估计仍可保留约 14 帧并通过门槛；
- 原 22 帧失败集合恢复为 22 帧有效包；
- 真正只有 6 帧的集合仍保持 6 帧并被拒绝；
- 47/54/52/53/55 帧等正常长包保持连续；
- 没有因 0.10 秒关联窗拆坏正常平飞包。

原始识别事件回放是诊断近似；飞行时仍以通过遥测对齐后的 `RECON_PACKET` 日志为准。

### 7.2 NX163 测试

- `uav_recon` 核心测试：`26 passed`；
- bridge 与 MissionManager 安全测试：`30 passed`；
- 比赛选择器测试：`10 passed`；
- `colcon build --symlink-install --packages-select uav_recon`：通过；
- 单靶完整隔离 dry-run：通过。

dry-run 闭环结果：

```text
12 帧 -> 1 个 finalized
-> 1 个 TargetCommand
-> manager_received
-> goto_requested
-> A/R/D 航线规划完成
```

测试保持 `dry_run_goto=true`、`allow_mission_upload=false`，没有向真实飞控上传航线，
也没有输出真实舵机信号。

## 8. 未改变的比赛与飞控规则

以下关键行为保持原样：

- `insert_wp_index=5`
- `resume_wp_index=10`
- 到达航点 5 或之后拒绝新目标
- 单靶 bridge 只转发一个合法 final
- 比赛模式必须得到三个不同非空 final，候选签名稳定 1 秒后选择唯一目标
- 数字模式取中位数，图案模式取比赛值最高者
- 静态定高使用 `legacy_geo_cluster`，不受飞行封包结束条件影响
- mission push、回读验证、`set_current` 和舵机命令实现未改

## 9. 部署与回退

NX163 本轮取包算法部署前备份：

```text
/home/nx163/uav_ros2_project/.codex_backups/20260917_2002_pixel_packet_v2/
/home/nx163/uav_ros2_project/.codex_backups/20260917_2020_git_parity/
```

此前同日 manifest、关停脚本和时钟修复也分别保留了时间戳备份。需要回退时应恢复
对应源码和 YAML 后重新执行：

```bash
cd /home/nx163/uav_ros2_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_recon
```

最终检查时 `youth-vision.service` 的 `ExecStart` 仍为
`start_single_target_fusion.sh image 0.00`，但 unit 状态为 `disabled/inactive`。
本次 SSH 账号对 `systemctl enable` 没有免密权限，因此没有擅自输入密码或绕过权限。
需要恢复上电自启时在 NX163 交互终端执行：

```bash
sudo systemctl enable youth-vision.service
```

离线时钟 timer 当前为 `enabled/active/waiting`。

## 10. 下一次实飞验收重点

1. 确认航点 5 前出现 `coordinate_finalized` 和 `coordinate_published`；
2. 检查正常 35 米平飞包帧数、拆包原因和中心像素轨迹；
3. 严格按闭环日志确认 `manager_received -> goto -> push -> set_current`；
4. 若目标仍只在航点 12 后入镜，应检查基础航线覆盖、相机朝向和现场摆放，不能用
   放松航点 5 安全门槛掩盖物理时序问题；
5. 比较最早爬升 final 与后续平飞 final 的坐标差，再决定单靶是否需要航点 4 前的
   最优候选选择机制。
