# NX163 与 NX164 同步原则

## 分支策略

- `main`：两台设备共同认可、可回退的稳定基线。
- `feature/<name>`：视觉、坐标或飞控功能开发。
- `hotfix/<name>`：现场小范围修复。
- 每次上机前创建带日期的 tag，并记录模型哈希和配置差异。

不要为 NX163、NX164 长期维护两条相互独立的源码分支，否则相机、坐标和飞控修复会逐渐丢失同步。

## 统一仓库上传入口

统一仓库已转移至组织：

```text
https://github.com/CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system.git
```

开发机本地仓库继续位于 `github_publish/CUADC-UAV-Recon-Fusion/`，但其 `origin`
必须使用上述组织地址。提交前先执行 `git fetch origin` 并确认本地分支没有落后；
只暂存本次确认过的源码路径，完成检查后再执行 `git push origin main`。

NX163 的 `/home/nx163/uav_ros2_project` 是独立飞控仓库，仓库根目录与统一仓库中的
`flight-ros2/` 子目录不同。禁止把 NX163 的 `origin` 直接改成统一仓库；应先把确认过的
飞控源码导入开发机的 `flight-ros2/`，再由开发机提交和推送。

## 必须分设备核验的内容

1. Linux 用户名和 `/home/nx163`、`/home/nx164` 路径。
2. 海康相机序列号与当前可枚举设备。
3. TensorRT、CUDA、JetPack 版本。
4. 三个 TensorRT engine 是否在本机生成并与配置对应。
5. systemd 的 `User`、`WorkingDirectory` 和 `ExecStart`。
6. FCU 串口或 UDP 地址、ROS 2 工作区和飞控配置。
7. 相机内参、安装方向及 `camera_offset_flu_m`。

## 推荐同步顺序

1. 在开发机完成拉取、代码审查和脚本语法检查。
2. 对比目标 Jetson 当前工作树，先做备份，不覆盖现场未提交修改。
3. 同步源码和配置，不同步 `build/install/log`。
4. 在目标 Jetson 本机构建 ROS 2 与原生程序。
5. 在目标 Jetson 本机生成或核验 TensorRT engine。
6. 先检查相机枚举，再验证纯视觉链路。
7. 之后验证侦察坐标链路；真实飞控入口必须在批准的安全环境验证。
8. 验证通过后打 tag，并更新设备部署记录。

## 禁止直接同步

- SSH 密钥、密码和网络凭据。
- 录像、日志、识别结果和临时 PID。
- 另一台 Jetson 生成的 TensorRT engine。
- ROS 2 的 `build/install/log`。
- 未确认来源的绝对路径和旧版备份文件。
