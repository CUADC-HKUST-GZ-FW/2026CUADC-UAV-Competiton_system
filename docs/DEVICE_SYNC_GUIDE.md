# NX163、NX164 与团队电脑同步指南

## 1. 唯一代码基线

唯一主仓库：

```text
git@github.com:CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system.git
```

仓库目录与两台 Jetson 的运行目录同名：

| Git 目录 | Jetson 运行目录 |
|---|---|
| `uav_ros2_project/` | `/home/<用户>/uav_ros2_project/` |
| `youth-vision-runtime/` | `/home/<用户>/youth-vision-runtime/` |

当前源码基线以 NX163 实际运行代码为准。NX164 只保留用户名、相机序列号、设备生成的 TensorRT engine、网络和串口等硬件差异，不再维护另一套源码。

## 2. 新笔记本首次配置

先安装 Git，并让 GitHub 组织管理员把队员加入具有写权限的团队。每名队员使用自己的 SSH 密钥，不共享私钥。

```bash
ssh-keygen -t ed25519 -C "姓名@CUADC"
```

把公钥加入个人 GitHub 账号后测试：

```bash
ssh -T git@github.com
git clone git@github.com:CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system.git
cd 2026CUADC-UAV-Competiton_system
```

Windows PowerShell、Git Bash、WSL、Linux 和 macOS 均使用同一个仓库地址。

## 3. 日常协作

开始任务：

```bash
git switch main
git fetch origin
git pull --ff-only origin main
git switch -c feature/姓名-任务-日期
```

完成后：

```bash
git status --short
git diff --check
git add <本次文件>
git commit -m "fix: 修改目的"
git fetch origin
git rebase origin/main
git push -u origin HEAD
```

在 GitHub 创建 Pull Request，经另一名队员检查和测试后合并。不要多人直接覆盖 `main`。

## 4. Jetson 首次接入统一仓库

分别登录 NX163 或 NX164：

```bash
ssh nx163@192.168.55.1
# 或 ssh nx164@192.168.55.2
```

在 Jetson 上创建独立的 Git 工作副本：

```bash
cd ~
git clone git@github.com:CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system.git
cd ~/2026CUADC-UAV-Competiton_system
```

实际运行目录继续保持：

```text
~/uav_ros2_project
~/youth-vision-runtime
```

不要把统一仓库的 `origin` 直接设置到旧的 `~/uav_ros2_project/.git`；统一仓库是多目录仓库，根目录不同。

## 5. Jetson 日常更新

无论队员使用哪台笔记本，SSH 登录后的命令相同。推荐使用统一入口，脚本会自己执行
`fetch` 和 `fast-forward`：

```bash
cd ~/2026CUADC-UAV-Competiton_system
./deploy/update_jetson.sh --check --all
./deploy/update_jetson.sh --apply --all
```

`--check` 只显示差异；`--apply` 才更新运行目录，并把被覆盖文件备份到
`~/deployment_backups/`。更新范围有三种：

```bash
# 飞行和视觉全部更新，按需重建原生视觉程序，并重建 ROS 2
./deploy/update_jetson.sh --apply --all

# 只更新飞行任务代码，并重建 ROS 2；保留本地视觉目录
./deploy/update_jetson.sh --apply --flight-only

# 只更新视觉代码，按需重建原生视觉程序，不重建 ROS 2
./deploy/update_jetson.sh --apply --vision-only
```

每种范围都应先把 `--apply` 换成 `--check` 预览。`--uav-only` 是
`--flight-only` 的兼容别名。未明确写范围时默认使用 `--all`。

部署脚本会：

- 只将选中的源码目录同步到实际运行目录；
- 在 NX164 上把部署副本中的 `/home/nx163/` 渲染为 `/home/nx164/`；
- 保留本机当前相机序列号；
- 不覆盖 engine、模型、录像、日志、识别结果及 ROS 2 的 `build/install/log`；
- 视觉 C++ 或 Makefile 变化、二进制缺失或哈希不一致时，在暂存目录重建
  `native/build/youth_vision_runner`，成功后才原子替换旧程序；
- 分别记录飞行与视觉代码来源以及最后部署的 Git commit。

查看设备当前部署版本：

```bash
cat ~/.local/state/cuadc-uav/deployed.env
```

## 6. 设备上临时修复

优先在 Git 工作副本创建 `hotfix/` 分支后修改，不要直接改运行目录：

```bash
cd ~/2026CUADC-UAV-Competiton_system
git switch main
git pull --ff-only origin main
git switch -c hotfix/姓名-问题-日期
```

提交、推送并完成检查后，再运行同步脚本。紧急情况下若先改了运行目录，必须先把差异复制回 Git 工作副本并提交，不能只留在一台 Jetson。

## 7. 每台设备必须单独核验

1. Linux 用户名与 HOME。
2. 海康相机序列号及枚举结果。
3. JetPack、CUDA、TensorRT 版本。
4. 三个 TensorRT engine 的文件名和 SHA-256。
5. systemd 的 `User`、`WorkingDirectory` 与 `ExecStart`。
6. FCU 串口或 UDP 地址、MAVROS system/component ID。
7. 相机内参、安装角和机体偏移量。
8. `insert_wp_index=5`、`resume_wp_index=10`。

## 8. 发布前检查

```bash
git status --short
git log -1 --oneline
git diff --check
```

然后依次执行源码测试、ROS 2 构建、相机枚举、纯视觉验证和隔离 dry-run。真实航线改写与舵机动作只能在批准的安全条件下验证。
