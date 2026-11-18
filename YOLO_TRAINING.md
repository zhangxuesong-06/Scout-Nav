# 本机 YOLOv8 GPU 训练环境

目标是让真实 Orbbec 相机识别整串实物葡萄模型。训练环境为
`/home/frank/scout/.venv-yolo-train`，与 ROS 视觉跟随的 `.venv-vision` 分开使用。

## 安装日志的结论

2026-11-17 的日志中，指定的 `torch==2.5.1` / `torchvision==0.20.1` CUDA 12.4
下载因网络超时失败；后续安装 `ultralytics==8.3.0` 时，pip 自动安装了较新的 Torch。
因此最终已有可用环境，不必重复下载整套 CUDA 依赖。

实际安装版本：

| 组件 | 版本 |
| --- | --- |
| Python | 3.10.12 |
| torch | 2.14.1+cu130 |
| torchvision | 0.29.2+cu132 |
| ultralytics | 8.3.0 |
| numpy | 1.26.4 |
| GPU | RTX 4070 Laptop，8 GB 显存 |

`typeguard` 缺失的提示来自终端继承的其他 ROS 工程路径；清除这些路径后，
训练环境自身的 `pip check` 通过。

## 每个训练终端

```bash
source /home/frank/scout/scripts/yolo_train_env.bash
python -m pip check
python -c 'import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

该入口清除继承的 ROS Python / 动态库路径、排除 `~/.local` 包并激活训练 venv。
应在专门的训练终端使用；随后不要再加载 `.bashrc`、ROS 或 `vision_env.bash`。

## 准备葡萄数据后训练

目前尚未准备真实葡萄训练集。先将图片和 YOLO 检测框标签整理成：

```text
/home/frank/scout/datasets/grapes/
  images/train/
  images/val/
  images/test/
  labels/train/
  labels/val/
  labels/test/
  data.yaml
```

`data.yaml` 内容：

```yaml
path: /home/frank/scout/datasets/grapes
train: images/train
val: images/val
test: images/test
names:
  0: grape
```

每个框的标签为 `0 center_x center_y width height`，坐标相对图片宽高归一化，
框住整串葡萄。不同拍摄批次划到不同集合，避免相邻视频帧同时出现在训练集和验证集。
公共数据和候选葡萄权重见 [GRAPE_RESOURCES.md](GRAPE_RESOURCES.md)。

**数据和标签整理完成后**，可从以下配置起步：

```bash
source /home/frank/scout/scripts/yolo_train_env.bash
cd /home/frank/scout/src/scout_visual_follow/models
yolo detect train \
  model=/home/frank/scout/src/scout_visual_follow/models/yolov8n.pt \
  data=/home/frank/scout/datasets/grapes/data.yaml \
  epochs=100 imgsz=640 batch=8 device=0 workers=2 cache=False \
  patience=20 seed=0 \
  project=/home/frank/scout/runs/grape name=grape_v1
```

这里切换到现有权重目录，是为了让内置 AMP 检查直接复用 `yolov8n.pt`，避免再次下载。

若显存不足，将 `batch=8` 降到 `4`，再降到 `2`。当前磁盘剩余约 23 GB，
先检查数据集体积，训练时保留 `cache=False`。

训练得到的 `runs/grape/grape_v1/weights/best.pt` 还需用独立拍摄批次和真实 Orbbec 视频
验收远距离、遮挡、反光与空背景误检；完成训练不等于识别已经稳定。

## 直接使用 Orbbec 彩色相机做实时识别

单独检测葡萄时，可以直接读取 Orbbec 的 UVC 彩色设备，不需要启动 ROS、深度驱动或 Docker。
当前这台相机的 RGB 序列号为 `CH28A5300TC`。先关闭使用该 RGB 设备的其他程序，再执行：

```bash
bash /home/frank/scout/scripts/grape_camera.bash
```

脚本自动选择唯一的 Orbbec RGB 设备；发现多台时要求显式传入 `--camera /dev/v4l/by-id/...`。
默认使用 `/home/frank/Downloads/wgisd-grape-cluster-detection-v1.pt`、GPU 0、
`imgsz=800`、`conf=0.25`，采集 640×480、15 FPS 的彩色流。
窗口按 `Q` / `Esc` 退出，按 `S` 保存原图和检测图到 `runs/grape_camera_live`；
`status.json` 每隔约 5 秒更新实际帧率和检测框。该脚本只做显示，不接入运动控制。

可调整阈值或更换模型：

```bash
bash /home/frank/scout/scripts/grape_camera.bash --conf 0.10
bash /home/frank/scout/scripts/grape_camera.bash --model /home/frank/Downloads/best.pt
```

2026-11-17 已在本机打开实时窗口并读取超过 400 帧，整体约 8 FPS。
首次画面可见模拟葡萄架，但 WGISD 权重在左侧背景出现误检，尚未稳定检出葡萄。

## 环境验证记录

基础检查已通过：独立环境依赖检查、CUDA 张量反向传播、GPU NMS、现有 YOLOv8n 权重
加载与 GPU 推理。

另用 4 张训练、2 张验证的合成矩形图片完成了 1 epoch 的端到端训练：
`imgsz=320`、`batch=2`、`device=0`、`workers=0`，AMP 检查、训练损失、验证、
权重保存及重新加载均通过。这只验证训练链路，不代表葡萄识别能力，也未验证正式数据集规模下的吞吐。
详细记录见 `log/yolo_training/environment_check.json` 和 `log/yolo_training/smoke.log`；
该目录下的测试权重类别为 `smoke_rectangle`，不能用于葡萄识别。

依赖快照：`log/yolo_training/requirements-validated.txt`。它记录当前环境；
不是要求重新安装的命令，也不是跨平台的完整锁文件。
