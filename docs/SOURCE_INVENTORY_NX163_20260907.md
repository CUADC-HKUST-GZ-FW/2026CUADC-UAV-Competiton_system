# NX163 源码导出清单

## 来源

- 设备：`nx163@192.168.55.1`
- 主机名：`nx163-desktop`
- 导出日期：`2026-09-07`
- 视觉根目录：`/home/nx163/youth-vision-runtime`
- 飞控入口软链接：`/home/nx163/uav_ros2_project`
- 飞控实际目录：`/home/nx163/jetson_fixedwing_deployment_20260810/jetson_fixedwing_deployment_20260810_payload/flight/uav_ros2_project`

## 视觉源码

已纳入：

- `scripts/`
- `configs/`
- `native/`，排除已编译目录
- `geo_bridge/`
- 根目录启动脚本、配置和技术说明

未纳入：

- `engines/`、`weights/`、`models/`、`onnx/`
- `logs/`、`recon_results/`、`overlays/`、`run/`
- `offline_inference/`、`collections/`、`staging/`、`backups/`

这些目录属于设备相关二进制、采集数据、运行输出或历史备份，不属于可维护源码。

## 飞控源码

已纳入：

- `src/`
- `scripts/`
- `config/`、`deploy/`
- `test/`、`test_data/`
- `wp/`、`param/`
- `README.md` 和任务交接说明

未纳入 ROS 2 的 `build/`、`install/`、`log/` 以及 MAVLink `tlog`。

原飞控仓库来源：

```text
git@github.com:CUADC-HKUST-GZ-FW/uav_ros2_project.git
```

导出时 HEAD：

```text
36f2e462f89663ea577d5178633428bd137a3f96
```

NX163 工作树在该 HEAD 之上包含未提交的最新修改。本仓库保存修改后的完整文件，并在 `docs/provenance/NX163_FLIGHT_WORKTREE_20260907.patch` 保留原始差异，避免退回旧远端版本。

## 审计结果

- 没有超过 10 MB 的待提交源码文件。
- 没有 `.engine`、`.pt`、`.onnx` 或视频文件。
- 未发现常见 GitHub Token、私钥或明文密码模式。
- 当前视觉和飞控生效配置没有 `/home/nx164` 路径残留。
- 历史说明和 SITL 开发脚本仍含旧开发机路径，不能视为 NX163 生效配置。

