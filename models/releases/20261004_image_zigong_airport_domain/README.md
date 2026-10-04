# 2026-10-04 自贡赛场域图案分类模型

本 release 是用户于 2026-10-04 明确批准采用的 13 类图案分类源模型。

## 文件

- `image_zigong_airport_domain_1004_best.pt`：PyTorch 源权重。
- `image_zigong_airport_domain_1004_best.onnx`：静态 `1x3x128x128`、opset 17 ONNX。
- `previous/`：此前部署的 0917 Gloo 图案模型 PT/ONNX 完整备份。
- `manifest.json`：新旧 SHA-256、训练来源、回归结果和双机 engine 命名规则。

## 关键结果

- 自贡运输机留出回放：`150029=46/47`、`152345=47/47`。
- 训练来源回放：`151217=16/16`。
- 最长连续“运输机误判为火箭兵”：现役模型 6 帧，新模型 0 帧。
- 历史回归：现役 `264/276`，新模型 `266/276`。
- real-empty：`20/20`。

## 部署

在目标 Jetson 的新鲜 Git clone 中运行：

```bash
./deploy/deploy_image_model_20261004.sh --check
./deploy/deploy_image_model_20261004.sh --apply
```

脚本只更新 `image_cls_engine`，不会修改 Pose、数字模型、NX163/NX164 相机参数或飞行任务参数。它根据登录用户名分别生成：

- NX163：`image_zigong_airport_domain_1004_best_fp16_nx163.engine`
- NX164：`image_zigong_airport_domain_1004_best_fp16_nx164.engine`

TensorRT engine 必须在各自 Jetson 本机生成，禁止跨设备复制。
