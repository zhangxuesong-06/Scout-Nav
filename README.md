# Scout-Nav

Scout Mini 差速底盘 + 固定 Orbbec DaBai DCW 的 ROS 2 Humble 工作空间。
已验证仿真、葡萄 RGB-D 识别跟随和架空底盘控制；RTAB-Map / Nav2 尚未完成实机验收。
Piper 控制不在本工程内。

## 克隆与编译

前提：**Ubuntu 22.04 x86_64、已安装 ROS 2 Humble Desktop 并配置 ROS 软件源**。
默认视觉环境使用 NVIDIA GPU，需先确保 `nvidia-smi` 正常；无 GPU 时启动传 `device:=cpu`。
以下整段可一次复制粘贴，安装依赖、创建独立视觉环境并编译测试；要求 `~/scout` 尚不存在。

```bash
bash <<'SCOUT_SETUP'
set -eo pipefail
[ -f /opt/ros/humble/setup.bash ] || { echo '请先安装 ROS 2 Humble Desktop'; exit 1; }
[ ! -e "$HOME/scout" ] || { echo '~/scout 已存在，请使用下方已有工作区的编译命令'; exit 1; }
sudo apt-get update
sudo apt-get install -y git python3-pip pkg-config libeigen3-dev \
  ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs

git clone https://github.com/zhangxuesong-06/Scout-Nav.git "$HOME/scout"
cd "$HOME/scout"
bash scripts/install_system_deps.bash

python3 -m venv --system-site-packages .venv-yolo-train
source scripts/yolo_train_env.bash
python -m pip install --upgrade pip
python -m pip install --timeout 120 --retries 10 \
  torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu124
python -m pip install --timeout 120 --retries 10 \
  numpy==1.26.4 opencv-python==4.11.0.86 ultralytics==8.3.0
python -m pip check

bash scripts/build.bash
bash scripts/test.bash
echo '完成。启动跟随：bash ~/scout/scripts/follow.bash'
SCOUT_SETUP
```

已有工作区只需：

```bash
cd ~/scout
bash scripts/build.bash
bash scripts/test.bash
```

仓库包含底盘/相机源码、仿真资源和权重，无需再次运行 `fetch_base.bash`。
相机权限不足时安装供应商 `src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules`
到 `/etc/udev/rules.d/`，重载 udev 后重新插拔相机。系统不需要另装 CUDA Toolkit 或 `nvcc`；
上述 PyTorch 命令安装 CUDA 运行时，显卡驱动需与其兼容。

## 实机葡萄跟随

先检查 `config/robot.yaml` 中固定相机序列号、Scout 专用 USB-CAN 序列号与 `can_interface`，
并将遥控器切到上位机/CAN 档。默认配置属于当前这台设备，不可直接套到另一台机器人。

```bash
bash ~/scout/scripts/follow.bash
```

该命令启动相机、安装 TF、Scout 驱动、检测跟随和运动出口；CAN 接口 DOWN 时会通过
`sudo` 配成 500 kbit/s。新鲜输入到齐后自动使能一次，选定目标才产生跟随运动。
窗口操作：**`i` → 输入目标 ID → 回车**；`c` 取消，`Esc` 或终端 `Ctrl+C` 结束整套节点。

参数使用 `参数:=值` 追加，例如：

```bash
bash ~/scout/scripts/follow.bash \
  confidence:=0.15 imgsz:=800 \
  max_linear_speed:=0.05 max_angular_speed:=0.15 \
  depth_speckle_size:=20 depth_min_valid_ratio:=0.15
```

以下为 **`follow.bash` 的有效默认值**（脚本会覆盖部分 launch 默认值）：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `confidence` | `0.10` | 检测置信度，越低越容易检出，也越容易误检 |
| `imgsz` | `800` | 推理输入尺寸，不是相机采集分辨率 |
| `min_box_area` | `200` | 最小检测框面积，像素² |
| `device` | `0` | GPU 编号；可用 `cpu` |
| `max_linear_speed` | `0.05` | 最大前进速度，m/s，同时限制跟随节点和运动出口 |
| `max_angular_speed` | `0.15` | 最大转向角速度，rad/s，同时限制两个节点 |
| `depth_speckle_size` | `20` | 相机小块深度过滤阈值 |
| `depth_min_valid_pixels` | `20` | 框中央测距区域所需的最少有效深度像素 |
| `depth_min_valid_ratio` | `0.15` | 同一区域所需的最低有效深度比例 |
| `show_window` | `true` | 显示识别窗口；无 GUI 时用 `false` |
| `target_id` | `-1` | 初始目标 ID；`-1` 表示未选择 |
| `model_path` | `~/scout/models/grapes/wgisd-grape-cluster-detection-v1.pt` | 本地权重文件 |
| `python_executable` | `~/scout/.venv-yolo-train/bin/python` | 视觉节点解释器 |
| `profile` | `~/scout/config/robot.yaml` | 硬件配置；也可通过 `SCOUT_PROFILE` 指定 |
| `auto_enable` | `true` | 启动时自动使能一次；设 `false` 后手动使能 |
| `allow_approximate_mount` | `true` | 接受配置中明确标记的近似相机外参，仅用于跟随测试 |
| `start_hardware` | `true` | 启动实机；设 `false` 会拒绝启动，不是预览模式 |

路径参数使用绝对路径或 `"$HOME/..."`。查看 launch 参数声明：

```bash
bash ~/scout/scripts/follow.bash --show-args
```

停车距离、恢复距离和前进增益目前在 `src/scout_visual_follow/scout_visual_follow/node.py`
的 `stop_distance=0.5`、`resume_distance=1.0`、`linear_gain=1.0`，尚不支持在上述命令后追加。
修改 `src/` 后重新编译并重启；修改 `config/robot.yaml` 后重启。命令行参数只影响本次运行。
无窗口时可在另一个已加载环境的终端选择目标：

```bash
ros2 param set /scout/visual_follower target_id 0
```

## 查看速度和停车原因

保持跟随运行，**每个新终端先加载环境**，再选择需要的查看命令：

```bash
source ~/scout/scripts/ros_env.bash
```

| 内容 | 查看命令 | 关键字段 |
| --- | --- | --- |
| 跟随算法候选速度 | `ros2 topic echo /scout/follow_cmd_vel` | `linear.x`、`angular.z` |
| 交给底盘驱动的指令速度 | `ros2 topic echo /scout/cmd_vel` | `linear.x`、`angular.z` |
| 底盘回传速度及状态 | `ros2 topic echo /scout/status --no-arr` | `linear_velocity`、`angular_velocity`、`control_mode`、`error_code` |
| 仅看回传速度的里程计形式 | `ros2 topic echo /scout/odom --field twist.twist` | `linear.x`、`angular.z` |
| 跟随停车原因及测距质量 | `ros2 topic echo /scout/follow_targets --field data` | `control_state`、`depth_valid_ratio`、`image_age_ms`、`processing_ms` |

线速度单位 m/s，正值前进；角速度单位 rad/s，正值左转。里程计中的速度来自同一份底盘反馈，
不是独立测量；架空时回传速度非零不表示车体有地面位移。候选速度非零而指令速度为零时检查运动出口；
指令非零而回传长期为零时检查 CAN、控制档和底盘状态。

| `control_state` | 含义 |
| --- | --- |
| `following` | 正在跟随 |
| `no_selection` / `target_missing` | 未选目标 / 选中的 ID 丢失 |
| `too_few_depth_pixels` / `sparse_depth` | 有效深度像素数量 / 比例不足 |
| `distance_hold` | 进入停车距离，或尚未达到恢复距离 |
| `stale_after_inference` | 推理结束时图像已过期 |
| `target_behind_base` | 目标不在车体前方 |

缺图像/处理超时或 TF 等异常也会在启动终端打印原因；完全没有新图像时，诊断话题可能停止更新。
运动出口打印 `Stale input: stopped and disabled` 表示已超时锁定，不会自动恢复。

```bash
# 停车并禁止继续运动
ros2 service call /scout/motion_enable std_srvs/srv/SetBool '{data: false}'
# 排除原因后，允许新的速度指令通过
ros2 service call /scout/motion_enable std_srvs/srv/SetBool '{data: true}'
```

## 其他启动方式

只看葡萄彩色识别，不接运动控制；此时先关闭占用相机的 ROS/Docker 进程：

```bash
bash ~/scout/scripts/grape_camera.bash --conf 0.10
```

支持 `--model 路径`、`--camera /dev/videoN`、`--conf 0.25`、`--imgsz 800`、`--device 0`；
这是普通命令行参数格式，不是 `参数:=值`。`Q` / `Esc` 退出，`S` 保存图片。

已有相机和安装 TF，只启动 RGB-D 检测跟随预览：

```bash
bash ~/scout/scripts/grape_follow.bash confidence:=0.15 show_window:=true
```

该入口仅发布候选速度；支持上表中的检测、速度、深度有效像素、窗口、目标、模型、解释器与
`profile` 参数，不支持 `depth_speckle_size`、`auto_enable`、`allow_approximate_mount`、`start_hardware`。

## Gazebo 仿真

编译后即可启动 ROS 2 Humble + Gazebo Fortress 仿真，无需连接实车或相机：

```bash
bash ~/scout/scripts/sim.bash  # 默认场景 clearpath_playpen
bash ~/scout/scripts/list_sim_worlds.bash  # 列出可用场景
bash ~/scout/scripts/sim.bash world:=orchard camera_pitch_deg:=20 rviz:=true
```

`camera_pitch_deg` 为相机向下俯角（默认 `0°`）；`x`、`y`、`yaw` 为初始位姿（默认 `0`、`-2`、`0`，单位米、弧度）。
可追加 `gui:=false` 关闭 Gazebo 窗口、`software_rendering:=true` 使用软件渲染、
`camera:=false` 关闭 RGB-D、`static_environment:=false` 让场景中的可移动道具参与物理仿真。
用 `bash ~/scout/scripts/sim.bash --show-args` 查看全部参数。

**视觉跟随：** 保持仿真运行并开启 RGB-D，在新终端执行：

```bash
source ~/scout/scripts/sim_env.bash
PYTHONNOUSERSITE=1 YOLO_OFFLINE=true ros2 run \
  --prefix "$HOME/scout/.venv-yolo-train/bin/python" scout_visual_follow visual_follow --ros-args \
  -p model_path:="$HOME/scout/src/scout_visual_follow/models/yolov8n.pt" \
  -p use_sim_time:=true -p depth_registered:=true -p show_window:=true \
  -p 'device:="0"' -p confidence:=0.20 -p min_box_area:=200.0 \
  -p cmd_vel_topic:=/scout_sim/cmd_vel
```

窗口中按 `i` → 输入检测框 ID → 回车开始跟随，`c` 取消。无 GPU 时改用 `-p device:=cpu`。
`sim_env.bash` 将仿真和实机隔离在不同的 ROS 域中；每个查看或控制仿真的终端都先加载它。
按需执行以下查看命令：

```bash
source ~/scout/scripts/sim_env.bash
ros2 topic echo /scout_sim/cmd_vel                 # 仿真指令速度
ros2 topic echo /scout/odom --field twist.twist   # 仿真里程计速度
ros2 topic echo /scout/follow_targets --field data # 跟随状态和停车原因
```

**手动试车：** 先退出视觉跟随，再向 `/scout_sim/cmd_vel` 发指令，按 `Ctrl+C` 后约 0.5 秒自动停车：

```bash
source ~/scout/scripts/sim_env.bash
ros2 topic pub -r 10 /scout_sim/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

**建图与自动跟随：** 先退出其他仿真。需要 RTAB-Map（克隆步骤已安装；已有工作区缺失时运行 `sudo apt install ros-humble-rtabmap-slam`）。

```bash
bash ~/scout/scripts/sim_mapping.bash                      # 无葡萄，开识别并绕圈建图
bash ~/scout/scripts/sim_mapping.bash grapes:=true rviz:=true # 加葡萄照片靶，验证自动切换
```

沿固定路线建图，连续识别到有深度的葡萄后自动跟随；丢失目标后回到未完成的航点。地图持续更新，数据库默认保存到 `~/scout/maps/sim-*.db`。
常用参数：`vision:=false` 只建图；`confidence:=0.35`、`imgsz:=640`、`device:=cpu` 调推理；
`stop_distance:=1.60 resume_distance:=1.80` 调停车/恢复距离（米）；`show_window:=true` 显示检测，`gui:=true` 显示 Gazebo。
用 `bash ~/scout/scripts/sim_mapping.bash --show-args` 查看全部参数。

```bash
source ~/scout/scripts/sim_env.bash
ros2 topic echo /scout_sim/mapping_state --field data # 任务状态、当前航点、速度和闭环编号
```

场景、话题和仿真检查详见 [SIMULATION.md](SIMULATION.md)。

## 建图与导航入口

均先加载 `ros_env.bash`。先实测并确认相机外参，导航还需实测 `robot.footprint`；
当前近似外参和空轮廓会阻止建图/导航启动。以下各条分别在独立终端运行：

```bash
ros2 launch scout_nav_bringup camera.launch.py soft_filter_speckle_size:=20
ros2 launch scout_nav_bringup mount.launch.py
ros2 launch scout_nav_bringup base.launch.py start_hardware:=true
ros2 launch scout_nav_bringup mapping.launch.py database:="$HOME/scout/maps/scout.db"
```

单独启动底盘前，接口 DOWN 时执行 `sudo ip link set can0 up type can bitrate 500000`。
完成建图后关闭建图节点，以 `mapping.launch.py localization:=true database:=同一数据库绝对路径` 重启定位，
再启动 `navigation.launch.py` 和 `motion_gate.launch.py mode:=navigation`。
这些 launch 均支持 `profile:=配置绝对路径`；运动出口默认禁止运动，需调用上述使能服务。
实机跟随与导航共用一个运动出口，不要同时启动。

## 权重与源码

`models/grapes/` 的四份葡萄权重约 52 MB，直接随 Git 提交，默认使用 detection-v1；
来源及 SHA-256 见 `models/grapes/sources.json`。通用/仿真权重位于
`src/scout_visual_follow/models/yolov8n.pt`，也随仓库提交。
`.gitignore` 排除构建目录、虚拟环境、日志、训练结果、数据集、地图和未完成下载，保留源码、仿真资源和权重。
源码与资产来源见 [SOURCE_REVISIONS.md](SOURCE_REVISIONS.md)、[NOTICE.md](NOTICE.md)；训练见 [YOLO_TRAINING.md](YOLO_TRAINING.md)。
