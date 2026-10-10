# 模型制品说明

本目录默认不提交训练过程中的 `.pt`、`.onnx` 或 TensorRT `.engine`。经人工明确批准的当前源模型可以放入 `models/releases/`，并同时提交来源、验收状态和 SHA-256。设备相关 TensorRT `.engine` 始终不进入 Git。

用户最新批准的 Pose 源模型：

- `releases/20261010_pose_umbrella1009_alpha30/arrow_pose_3pt_umbrella1009_alpha30.pt`
- `releases/20261010_pose_umbrella1009_alpha30/arrow_pose_3pt_umbrella1009_alpha30.onnx`
- 该版本针对 10 月 9 日蓝伞稳定错检加入完整帧困难负样本，并以 `alpha=0.30` 保守插值保护历史召回；蓝伞最长连续误检由 `7/5/12` 降为 `6/5/10` 帧。
- 推荐连续帧门槛为 **12 帧**：8 帧仍会放过 10 帧蓝伞误检，16 帧则会比 12 帧少保留 2 个历史真图案标靶包。
- 同目录 `previous/` 保存 `positive1008` PT/ONNX。该版本已获准上传 GitHub，但尚未完成 NX163/NX164 各自 TensorRT 构建与全链路实机验收。

上一份 GitHub Pose 发布：

- `releases/20261009_pose_positive1008/arrow_pose_3pt_positive1008.pt`
- `releases/20261009_pose_positive1008/arrow_pose_3pt_positive1008.onnx`
- 该版本在 75 段全帧率窗口的 8 帧门槛下达到 `37/37` 真标靶和 `4/36` 非标靶误包，优于此前发布的 `alpha_05` 的 `33/37` 与 `14/36`。
- 同目录 `previous/` 保存发布前的 `alpha_05` PT/ONNX；`manifest.json` 记录训练来源、回归、哈希和 10 月 9 日后续主动学习的已知取舍。
- 该版本已获准上传 GitHub，但尚未完成 NX163/NX164 各自 TensorRT 构建与全链路实机验收。两台设备必须分别生成带 `_nx163`、`_nx164` 后缀的 engine，禁止跨设备复制。

上一份 GitHub Pose 发布：

- `releases/20261007_pose_airport_alpha05/arrow_pose_3pt_airport1006_alpha05.pt`
- `releases/20261007_pose_airport_alpha05/arrow_pose_3pt_airport1006_alpha05.onnx`
- 该版本是从 0930 基线向机场多源数据微调权重插值 5% 的 `alpha_05`；用户于 2026-10-07 明确决定采用。
- 该版本已发布源权重，但尚未完成 NX163/NX164 各自 TensorRT 构建与全链路实机验收。两台设备必须分别生成带 `_nx163`、`_nx164` 后缀的 engine，禁止跨设备复制。
- 同目录 `previous/` 保存发布前生产基线，可立即回退；`manifest.json` 保留自动筛选未推荐该候选的回归事实与人工采用决定。

上一版已完成双机验收的 Pose 基线：

- `releases/20260930_pose_gap0930/arrow_pose_3pt_gap0930.pt`
- `releases/20260930_pose_gap0930/arrow_pose_3pt_gap0930.onnx`
- 该版本已在 NX163/NX164 部署并完成无丢帧回放、单靶和赛事三靶全链路验收；同目录 `previous/` 保存 0902 Pose PT/ONNX 回退件。

当前图案分类源模型：

- `releases/20261004_image_zigong_airport_domain/image_zigong_airport_domain_1004_best.pt`
- `releases/20261004_image_zigong_airport_domain/image_zigong_airport_domain_1004_best.onnx`
- 同目录 `previous/` 保存此前部署的 0917 Gloo 图案模型 `.pt/.onnx`，`manifest.json` 保存新旧哈希和离线验收结果。
- NX163 与 NX164 必须分别在本机生成带各自设备名后缀的 TensorRT engine；部署入口为 `deploy/deploy_image_model_20261004.sh`。
- 旧 release `releases/20260917_image_real_blank_gloo/` 原样保留，可用于审计与回退。

最新图案分类候选：

- `releases/20261006_image_tanktruck_candidate/image_zigong_tanktruck_1006_candidate.pt`
- `releases/20261006_image_tanktruck_candidate/image_zigong_tanktruck_1006_candidate.onnx`
- 该候选修复了现有截图中的坦克到卡车误判并通过历史回归，但截图参与了训练，仍需独立的新坦克/卡车视频验收；在此之前当前生产模型仍为 1004 版本，不得直接部署候选。

用户最新批准的图案分类源模型：

- `releases/20261009_image_flight1009_alpha70/image_cls_flight1009_alpha70.pt`
- `releases/20261009_image_flight1009_alpha70/image_cls_flight1009_alpha70.onnx`
- 该版本针对 10 月 9 日实飞中的多旋翼到运输机、轰炸机到防空炮混淆进行保守修复；历史验证为 `3118/3120`，独立实飞测试由 1006 基线的 `65/96` 提升到 `88/96`。
- 用户已批准采用和发布。NX163/NX164 尚需分别在本机生成各自 engine 并完成无桨及全链路验收；1006、1007 历史版本完整保留。

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
