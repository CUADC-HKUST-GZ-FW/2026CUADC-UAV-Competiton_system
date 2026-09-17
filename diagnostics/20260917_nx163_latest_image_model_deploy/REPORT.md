# NX163 最新图案分类模型部署记录

日期：2026-09-17

## 部署内容

- 图案分类权重：`image_real_blank_gloo_best.pt`
- PT SHA256：`3aeef18d24551f5beba6d25f0537055267a15e31cd3ecae397fdbaaa7dd2497a`
- ONNX SHA256：`9d38de2053e0c15809f5c5a36f1a37e4f8c2ddc0773d4f76b52081b4db90b03d`
- NX163 FP16 engine：`image_real_blank_gloo_best_fp16_nx163.engine`
- engine SHA256：`2a1ab4fcb6f7aa69a7a8400cefe143ca2dabf8ea834f2da30224c93818e49b75`
- engine 大小：`4,366,380 bytes`
- TensorRT：`10.3.0.30-1+cuda12.5`
- CUDA：`12.6`
- L4T：`R36.5.0`

Pose 与数字分类模型没有改变。活动配置已切换到：

```text
/home/nx163/youth-vision-runtime/engines/image_real_blank_gloo_best_fp16_nx163.engine
```

## 功能验证

1. TensorRT 构建通过，输入 `1x3x128x128`，输出 `1x13`。
2. `trtexec` 平均 GPU 推理延迟约 `0.563 ms`。
3. 12 张冻结实拍裁剪的 ONNX/TensorRT Top-1 为 `12/12` 一致，最大概率绝对误差 `0.02302474`。
4. 海康相机 `DB1225037` 连续出帧，实测约 `60.6 FPS`。
5. 网页推理服务正常启动，`http://127.0.0.1:8000/` 返回完整页面。
6. 定高侦察入口成功同时启动视觉、MAVROS、地理解算和比赛选择器。
7. 侦察、桥接、任务安全和比赛选择器测试合计 `91 passed`。
8. 单标靶安全 dry-run 完成 `12 frames -> finalized -> bridge -> manager_received -> goto_requested -> route plan`；`allow_mission_upload=false`，没有真实航线上传和舵机动作。
9. 修复 `youth-vision.service` 的前台守护：`UAV_FOREGROUND=1` 下主进程和视觉、ROS/MAVROS、比赛选择器三个子进程持续存活；发送 `TERM` 后全部干净退出。

## 已知限制

这份模型在 2026-09-16 的全新四方向空标靶视频中没有通过精度门槛：

- 总体空标靶召回：`63.04%`；
- 朝下空标靶召回：`1.08%`；
- 朝左空标靶召回：`56.12%`；
- 主要误判为直升机和多旋翼。

因此，本次结论是“模型文件与软件链路可正常运行”，不是“比赛识别精度已验收”。

实机侦察测试还观察到飞控拒绝提高 `LOCAL_POSITION_NED` 发布频率，并出现约 `18-21 ms` 的 timesync RTT 警告；该问题与本次分类模型替换无关，但后续动态坐标精度验证必须继续关注。

## 回退

备份目录：

```text
/home/nx163/youth-vision-runtime/backups/20260917_latest_image_model_before/
```

回退时恢复该目录中的 `youth_pipeline.yaml`。旧 engine 未删除，仍位于：

```text
/home/nx163/youth-vision-runtime/engines/image_cls_fourway_balanced13_0903_night_v2_best_fp16_nx163.engine
```

若只回退开机守护行为，再恢复：

```text
/home/nx163/youth-vision-runtime/backups/20260917_latest_image_model_before/competition_selected.sh.before_foreground_fix
```

一致性审计文件：

- `compare_parity.py`
- `parity.json`
