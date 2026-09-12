# 团队开发约定

## 唯一主仓库

```text
git@github.com:CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system.git
```

`main` 只保存团队已经验证的共同基线。NX163、NX164 和个人电脑不维护长期独立分支。

## 每次开始工作

```bash
git switch main
git fetch origin
git pull --ff-only origin main
git switch -c feature/姓名-任务-日期
```

不要直接在 `main` 上试验。一个分支只处理一个问题，避免把模型、视觉和飞控无关改动混在同一提交。

## 提交与共享

```bash
git status --short
git diff --check
git add <本次修改的文件>
git commit -m "fix: 简述修改目的"
git fetch origin
git rebase origin/main
git push -u origin HEAD
```

随后在 GitHub 创建 Pull Request。至少由另一名队员检查后再合并到 `main`。

## 冲突规则

- 不使用 `git reset --hard`、`git push --force` 覆盖队友工作。
- 有冲突时先 `git status`，逐文件确认；不清楚的冲突由原作者共同处理。
- 录像、日志、TensorRT engine、密码、SSH 密钥和 ROS 2 编译目录不进入 Git。
- `uav_ros2_project/` 涉及航线改写与载荷控制，合并前必须通过测试；实机前还必须拆桨或按批准的安全流程验证。

## Jetson 上的工作位置

Git 工作副本与实际运行目录分开：

```text
~/2026CUADC-UAV-Competiton_system/   # 拉取、编辑、提交和推送
~/uav_ros2_project/                 # 实际 ROS 2 运行目录
~/youth-vision-runtime/             # 实际视觉运行目录
```

不要从实际运行目录直接推送。先在 Git 工作副本修改并提交，再执行：

```bash
./deploy/sync_local_jetson.sh --check
./deploy/sync_local_jetson.sh --apply
```

这样 NX163 与 NX164 使用同一提交，同时保留各自的相机序列号、engine、日志和编译产物。
