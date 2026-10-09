# 2026-10-09 positive1008 Pose 模型

这是以 `20260930_pose_gap0930` 为训练基线，加入 10 月 7 日赛场完整帧困难负样本和真实箭头正样本后得到的三点 Pose 模型。用户于 2026-10-09 明确要求将 `positive1008` 发布到 GitHub。

## 文件

- `arrow_pose_3pt_positive1008.pt`：PyTorch 源权重。
- `arrow_pose_3pt_positive1008.onnx`：静态 `1x3x768x1024`、opset 17、三关键点 ONNX。
- `previous/`：上一份 GitHub Pose 发布 `20261007_pose_airport_alpha05` 的 PT/ONNX 回退件。
- `manifest.json`：哈希、训练来源、回归结果、审批和部署状态。

## 回归摘要

75 段全帧率视频窗口中，相对已发布的 `alpha_05`：

| 连续帧门槛 | 真标靶 alpha_05 | 真标靶 positive1008 | 非标靶误包 alpha_05 | 非标靶误包 positive1008 |
|---:|---:|---:|---:|---:|
| 8 | 33/37 | 37/37 | 14/36 | 4/36 |
| 12 | 30/37 | 34/37 | 10/36 | 3/36 |
| 16 | 26/37 | 32/37 | 6/36 | 1/36 |
| 20 | 20/37 | 26/37 | 6/36 | 1/36 |
| 24 | 17/37 | 25/37 | 3/36 | 1/36 |

`target_empty` 在 8 至 24 帧门槛下保持 `2/2`。旧困难集盲测保留 `7/8` 个正样本帧和 `7/9` 个实例，历史 Pose mAP50 为 `0.984099`。

10 月 9 日后续主动学习再次提高了单帧召回，但在 8 帧门槛下会把非标靶误包从 `4/36` 增加到 `9/36`，没有全面优于本版本，因此没有替换 `positive1008`。

## 双机部署约束

同一 ONNX 可用于两台设备，但 TensorRT engine 必须分别在目标 Jetson 本机生成：

- NX163：`arrow_pose_3pt_positive1008_fp16_opt3_nx163.engine`
- NX164：`arrow_pose_3pt_positive1008_fp16_opt3_nx164.engine`

禁止在 NX163 与 NX164 之间复制 engine。发布仅代表源模型进入 GitHub；目前没有宣称已完成双机 TensorRT 构建或全链路实机验收。
