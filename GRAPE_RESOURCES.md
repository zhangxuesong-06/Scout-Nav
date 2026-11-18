# 葡萄检测权重与数据集筛选：2026-11-17

目标：架子上的真实 Orbbec 相机识别实物葡萄模型，按整串葡萄检测。
用户照片为红紫色、表面光亮的长串模型；资源无需与其完全相同。

## 当前跟随选择：WGISD detection-v1

用户在真实相机下试用后选择继续使用 `wgisd-grape-cluster-detection-v1.pt`，
不再以 YOLO11s 作为当前跟随候选。两张静态照片上的分数不代表实时跟随效果。
权重已复制至 `/home/frank/scout/models/grapes/wgisd-grape-cluster-detection-v1.pt`，
SHA-256 与 Downloads 原件一致，记录在 `models/grapes/sources.json`。

葡萄 RGB-D 跟随入口：`bash /home/frank/scout/scripts/grape_follow.bash`。
默认 conf=0.10、imgsz=800、最小框面积 200 像素、GPU 0，手动选择目标 ID；
仅输出候选速度，要求另行启动已对齐的 ROS RGB-D 相机与安装 TF。
原来的通用/仿真 COCO 入口和默认 2000 像素过滤保留。
RGB 检测入口 `grape_camera.bash` 也默认使用这份项目内权重。

## 第二轮搜索与静态实测记录

本轮找到并完整下载了 3 套额外权重，均在现有训练 venv 中成功加载并完成 GPU 推理。
每套使用两张用户照片和一张此前保存的 Orbbec 640×480 葡萄架截图测试，
统一 `imgsz=800`，分别检查 `conf=0.25`、`conf=0.10`。
测试仅反映这三张图片，不能作为总体准确率或视频稳定性指标。

| 模型 | 已核对的类别与类型 | 本机实测 |
| --- | --- | --- |
| Sandeep YOLO11s | `green`、`mix`、`purple`、`stem`；检测 | 两张近景照片均以单个 `purple` 框覆盖葡萄主体，置信度约 0.893 / 0.926；800 尺寸下相机远景漏检 |
| VEDAVARSHAN YOLOv8n | `unripe`、`ripe`、`rotten`；检测 | 第一张在 0.10 下框到葡萄但标签为 unripe；第二张框明显偏大，远景把大片背景误判为葡萄 |
| deepanik YOLO11n-seg | 数字类别 `0`、`1`、`2`；分割 | 三张图片在两个阈值下均无框或掩码 |

### 3. sandeepiitism/GrapesDetection_yolov11：静态近景候选

- 项目：https://github.com/sandeepiitism/GrapesDetection_yolov11
- 直接下载：https://raw.githubusercontent.com/sandeepiitism/GrapesDetection_yolov11/main/best.pt
- 大小：19,184,787 字节；完整下载，Git blob SHA 与仓库一致，ZIP CRC 校验通过。
- 本地：`/home/frank/scout/models/grapes/sandeep_yolo11s.pt`。
- checkpoint 确认为 YOLO11s 检测模型，保存版本 Ultralytics 8.3.59；现有 8.3.0 已成功加载。
- 类别：`0: green`、`1: mix`、`2: purple`、`3: stem`。接入导航时应区分葡萄类和 stem，
  不能把四类全当作葡萄串。现有 RGB 预览脚本显示全部类别。
- 仓库有 MIT LICENSE；checkpoint 另含 Ultralytics AGPL-3.0 说明，训练数据授权未进一步确认。
- 两张照片 `conf=0.25` 下的可视化：`runs/grape_test/sandeep_comparison.jpg`。
- 相机旧截图追加检查：`imgsz=1280, conf=0.10` 仍无框；`imgsz=1600, conf=0.10`
  在右上葡萄处出现一个 `purple` 框，置信度约 0.120。此弱检测还不足以证明远距离视频稳定。
- 记录：`runs/grape_test/sandeep_results.json`、`sandeep_camera_sizes.json`。

先退出原有预览窗口，再运行此候选，建议先近距离观察：

```bash
bash /home/frank/scout/scripts/grape_camera.bash \
  --model /home/frank/scout/models/grapes/sandeep_yolo11s.pt \
  --conf 0.25
```

复现远景截图的参数可用 `--conf 0.10 --imgsz 1600`；这会增加推理计算量，
实时帧率尚未测量。插值放大输入也不能恢复原始 640×480 画面中不存在的细节。

### 4. VEDAVARSHAN/grape-counting-ripeness-detection：成熟度模型

- 项目：https://github.com/VEDAVARSHAN/grape-counting-ripeness-detection
- 直接下载：https://raw.githubusercontent.com/VEDAVARSHAN/grape-counting-ripeness-detection/main/runs/detect/train/weights/best.pt
- 大小：6,236,330 字节；Git blob SHA、ZIP CRC 已验证完整。
- 本地：`/home/frank/scout/models/grapes/veda_yolov8n_maturity.pt`。
- checkpoint 为 YOLOv8n、Ultralytics 8.3.160，三类与仓库 data.yaml 一致。
- 数据配置注明 Roboflow `projects-hbafr/g2grape-li162` v2、CC BY 4.0；checkpoint 含 AGPL-3.0 说明。
- 当前模型会以很大的框覆盖背景，不推荐替换正在使用的 WGISD 模型；
  实物模型也不能用于验证真实葡萄成熟度判断。
- 可视化：`runs/grape_test/veda_contact_sheet.jpg`；记录：`runs/grape_test/veda_results.json`。

### 5. deepanik/Grape-seg：葡萄实例分割

- 项目：https://github.com/deepanik/Grape-seg
- 直接下载：https://raw.githubusercontent.com/deepanik/Grape-seg/main/paper_results/training_runs/yolov11/weights/best.pt
- 大小：6,089,508 字节；Git blob SHA、ZIP CRC 已验证完整。
- 本地：`/home/frank/scout/models/grapes/deepanik_yolo11n_seg.pt`。
- checkpoint 为 YOLO11n-seg、Ultralytics 8.4.24；现有环境已成功加载。
- data.yaml / 权重只有数字类别，类别语义仍不明确。本轮三张图均漏检，暂不优先。
- 仓库还公开了 YOLO12 / YOLO26 权重，本轮未下载或测试这些版本。
- checkpoint 含 AGPL-3.0 说明，项目及数据授权未进一步确认。
- 记录：`runs/grape_test/deepanik_results.json`。

第二轮三份模型及当前 WGISD 权重的来源、大小、SHA-256 记录在 `models/grapes/sources.json`，
第二轮下载还记录了 Git blob SHA。
搜索 API、配置及元数据证据位于 `research/grape/search_round2`。

## 优先试用的权重

### 1. El1z1um/yolov8_grapes：YOLOv8n，单类 grape

- 项目：https://github.com/El1z1um/yolov8_grapes
- 权重页面：https://github.com/El1z1um/yolov8_grapes/blob/main/runs/detect/train90/weights/best.pt
- 直接下载：https://raw.githubusercontent.com/El1z1um/yolov8_grapes/main/runs/detect/train90/weights/best.pt
- 文件大小：6,249,272 字节，约 6.25 MB。
- 数据配置：https://github.com/El1z1um/yolov8_grapes/blob/main/grapes.yaml
  明确 `nc: 1`、`0: grape`。
- 训练配置：https://github.com/El1z1um/yolov8_grapes/blob/main/runs/detect/train90/args.yaml
  指定 `task: detect`、`model: yolov8n.pt`、100 epochs、imgsz=800。
- 仓库还有训练曲线、CSV 和验证预测图。未独立复核其指标和数据划分。
- 仓库未提供清楚的 LICENSE；未确认数据来源及自定义权重的再分发/商业授权。
- 候选用途：第一轮葡萄检测试用。用户已下载完整文件至 `/home/frank/Downloads/best.pt`，
  文件大小一致，ZIP CRC 校验通过；SHA-256 为
  `8a2d6723341e4169966c4f6a775fa13bce12af2c857952ec351a15866c7aaa9b`。
- 2026-11-17 本机 GPU 训练环境（Ultralytics 8.3.0 / Torch 2.14.1）已成功加载，
  类别为 `{0: grape}`。对用户的 `IMG_20261117_225035.jpg` 使用 `imgsz=800`，
  分别以 `conf=0.25`、`conf=0.10` 推理，均无检测框。
  随后对第二张 `IMG_20261118_011453.jpg` 使用相同参数测试，两个阈值下同样没有检测框；
  结果记录在 `/home/frank/scout/runs/grape_test/photo2_results.json`。
  这是两张图片的漏检结果，尚未在真实 Orbbec 视频上验收。
  输出在 `/home/frank/scout/runs/grape_test/photo` 和 `photo_conf010`。

### 2. Emerson-MT/yolov8-grape-cluster-yield：WGISD 葡萄串 YOLOv8s

- 项目：https://github.com/Emerson-MT/yolov8-grape-cluster-yield
- 权重页面：https://github.com/Emerson-MT/yolov8-grape-cluster-yield/blob/main/models/wgisd-grape-cluster-detection-v1.pt
- 直接下载：https://raw.githubusercontent.com/Emerson-MT/yolov8-grape-cluster-yield/main/models/wgisd-grape-cluster-detection-v1.pt
- 文件大小：22,539,215 字节，约 22.54 MB。
- 静态检查了权重 ZIP 中第一个完整 `data.pkl` 成员的 pickle 操作码，未反序列化、未执行模型。
  元数据为 `DetectionModel`、`model: yolov8s.pt`、`data: wgisd-yolo/wgisd.yaml`、
  `nc: 1`、`names: {0: uva_bbox}`、训练配置 100 epochs；`uva` 是葡萄。
- 保存时 Ultralytics 版本为 8.3.229，比现有环境的 8.3.0 新。
  用户现已下载完整文件至 `/home/frank/Downloads/wgisd-grape-cluster-detection-v1.pt`，
  大小一致且 ZIP CRC 校验通过，SHA-256 为
  `7653f0cd2157e379522f2074f740ea906172b3512ffdcb674fbcb3a1b59ae8f2`。
- 本机训练环境 Ultralytics 8.3.0 / Torch 2.14.1 已成功加载并完成 GPU 推理，
  类别为 `{0: uva_bbox}`。两张用户照片均以 `imgsz=800`、`conf=0.25/0.10` 测试：
  第一张始终无框；第二张在 0.25 下出现 3 个框，目视检查只有约 0.49 的框覆盖葡萄下半串，
  另两个约 0.50、0.47 的框落在背景箱体上。在 0.10 下增加到 7 个框，仍有局部框和背景误检。
  结果见 `/home/frank/scout/runs/grape_test/wgisd_results.json` 和对应输出图片。
  当前两张照片的表现不足以支持稳定的整串检测。
- 检查结果：`research/grape/wgisd_checkpoint_metadata.json`。
- checkpoint 中写有 Ultralytics AGPL-3.0 许可说明；项目自身未提供清楚的 LICENSE，
  WGISD 数据另受 CC BY-NC 4.0 约束。
- 候选用途：与 nano 版本对比葡萄串检测效果，或以其训练数据路线重训自己的模型。

### 备选：包含葡萄的多水果模型

- 项目：https://github.com/apurboislam/fruit-detection-model
- Release：https://github.com/apurboislam/fruit-detection-model/releases/tag/v1.0
- `best_small.pt` 22,500,195 字节；`best.pt` 409,632,932 字节。
- 作者说明为 YOLOv8，包含 Apple、Banana、Orange、Grapes、Strawberry。
  Release API 已确认这两个附件存在。第二轮下载 `best_small.pt` 超时，仅保留约 1.9 MB 的
  `.part` 文件；从首个完整 metadata 成员静态核对为 YOLOv8s，类别顺序为
  `Orange`、`Banana`、`Apple`、`Grapes`、`Strawberry`，保存版本为 Ultralytics 8.3.237。
  该部分文件不能用于推理，尚未完成照片测试。
- README 中的准确率和 FPS 是作者报告，不能视为在本项目中的效果。
- 仓库未提供清楚的 LICENSE。

## 数据集

### 1. Embrapa WGISD：来源清楚的葡萄串数据

- 官方项目、说明及下载：https://github.com/thsant/wgisd
- 数据目录：https://github.com/thsant/wgisd/tree/master/data
- 300 张真实葡萄园 RGB 图片，5 个葡萄品种，约 4,400 串葡萄检测框。
  README 不同段落对总框数写作 4,431 / 4,432，实际训练应以标签计数为准。
- 检测标签已经是 YOLO 格式：`0 center_x center_y width height`，坐标归一化。
  137 张图片还带有分割 mask，但 `.npz` mask 不能直接当作 YOLO 分割多边形标签。
- 作者提供 242 张训练/验证、58 张测试的划分。重新训练时应从训练部分划验证集，保留测试集。
- 许可：CC BY-NC 4.0，署名、非商业；商业使用需联系数据提供方。
- 适用：学习葡萄串的外形、颜色、遮挡变化；葡萄园背景与室内仿真水果有差异。

### 2. Lightly 水果检测数据：含葡萄和其他水果

- 镜像：https://github.com/lightly-ai/dataset_fruits_detection
- 原始来源：https://www.kaggle.com/datasets/lakshaytyagi01/fruit-detection
- README 报告共 8,479 张图片；这是六类水果总数，不是葡萄图片数量。
- 配置：https://github.com/lightly-ai/dataset_fruits_detection/blob/main/data.yaml
  类别为 Apple、Banana、Grape、Orange、Pineapple、Watermelon，Grape 的原编号是 2。
- YOLO 检测格式，现成 train/valid/test 目录；README 说明图片被拉伸到 640×640。
- 镜像仓库 LICENSE 为 CC0-1.0；镜像说明注明数据来自 Kaggle，正式分发时仍应核对原始数据条款。
- 适用：补充葡萄外观和其他水果干扰样本，具体葡萄样本质量及占比尚未审核。
- 若训练成单类模型：只将葡萄的 class=2 改为 0，并删除其他类别的标签行；不能把所有类别都改成葡萄。

## 针对本项目的使用顺序

1. 先用 YOLOv8n 葡萄权重测试用户照片和新采集的 Orbbec 图片，再与 YOLOv8s 候选对比。
2. 重点看远距离漏检、葡萄部分遮挡、光亮塑料反光、无葡萄背景误检。
3. 公共模型不足时，先采集 100～200 张有变化的实物图片做小规模微调；
   这个数量是起步试验，不能保证稳定效果。保留不同拍摄批次的新视频做独立验收。
4. 公开葡萄园照片可作为补充，最终仍以真实 Orbbec 相机与室内场景的验证结果选模型。
5. 通用跟踪入口默认过滤面积小于 2000 像素的框；葡萄入口单独降到 200 像素。
   小框也需要足够有效深度才能产生候选速度，远处漏检需结合原始检测和深度一起检查。

## 本次验证范围

已通过 GitHub API、原始配置和数据说明核对资源文件、类别、格式与许可证信息。
网络对 raw.githubusercontent.com 及 Hugging Face 的访问不稳定。前两个权重现均已由用户下载完整，
并在两张用户照片上完成上述 GPU 推理和输出图片的目视检查。
WGISD 已在真实 Orbbec RGB 预览中运行，用户调整环境后能在 0.25 下检出中间葡萄，
后按用户要求切换到 0.10。第二轮三套新模型的本机量化记录来自照片和保存的相机截图；
用户随后试用并选择 WGISD 作为跟随模型。现已增加对应 ROS 跟随启动入口，尚未通过实机运动验收。
