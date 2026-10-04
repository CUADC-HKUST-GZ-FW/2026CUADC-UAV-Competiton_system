# 模型制品说明

本目录默认不提交训练过程中的 `.pt`、`.onnx` 或 TensorRT `.engine`。经人工明确批准的当前源模型可以放入 `models/releases/`，并同时提交来源、验收状态和 SHA-256。设备相关 TensorRT `.engine` 始终不进入 Git。

当前图案分类源模型：

- `releases/20261004_image_zigong_airport_domain/image_zigong_airport_domain_1004_best.pt`
- `releases/20261004_image_zigong_airport_domain/image_zigong_airport_domain_1004_best.onnx`
- 同目录 `previous/` 保存此前部署的 0917 Gloo 图案模型 `.pt/.onnx`，`manifest.json` 保存新旧哈希和离线验收结果。
- NX163 与 NX164 必须分别在本机生成带各自设备名后缀的 TensorRT engine；部署入口为 `deploy/deploy_image_model_20261004.sh`。
- 旧 release `releases/20260917_image_real_blank_gloo/` 原样保留，可用于审计与回退。

当前数字分类源模型：

- `releases/20261004_digit_large101_v4_conservative/digit_cls_large101_1003_v4_conservative_best.pt`
- `releases/20261004_digit_large101_v4_conservative/digit_cls_large101_1003_v4_conservative_best.onnx`
- 同目录 `previous/` 保存此前部署的 0903 数字模型 `.pt/.onnx`，`manifest.json` 保存新旧哈希和已知回归。
- NX163 与 NX164 必须分别在本机生成带各自设备名后缀的 TensorRT engine；部署入口为 `deploy/deploy_digit_model_20261004.sh`。

`NX163_MODEL_ARTIFACTS.sha256` 记录 NX163 当前视觉配置实际引用的三个 engine 的文件名、大小和 SHA-256。

部署到 NX164 时应使用训练源权重在 NX164 本机重新生成 engine，并重新登记哈希。不要只因两台设备目录结构相似，就复制 NX163 的 engine。

每次模型更新至少记录：

- Pose、图案分类、数字分类模型版本；
- 训练来源与验收报告；
- `.pt` 或 `.onnx` 的 SHA-256；
- 目标设备的 TensorRT、CUDA 和 JetPack 版本；
- 本机生成 engine 的 SHA-256；
- 回退模型及其哈希。
