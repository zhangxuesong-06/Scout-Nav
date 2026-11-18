# 源码来源与版本

- Orbbec：`https://github.com/orbbec/OrbbecSDK_ROS2.git`
  - 本机仓库：`/home/frank/ros2_ws/src/OrbbecSDK_ROS2`
  - 提取 ref：`origin/main`
  - commit：`c780bc110826536500dd706baa1eca8392ed39cb`
  - 包版本：`orbbec_camera 1.5.15`，SDK `1.10.35`
  - 使用 `git archive` 提取独立副本，没有修改本机原仓库的 `v2-main` checkout。
- Scout：官方 `agilexrobotics/scout_ros2` 的 `humble` 分支。
  - commit：`bf0110ff44cc70cdda2931d69f0e2669a3e26b9d`。
  - 从该 commit 的官方 archive 提取，SHA256：
    `709b48ae3ffa13568dcebf9bb91cb1af5d2e86a34aaf70b39150bbfd3ee38b38`。
  - 本地修改 `scout_messenger.hpp`：50 Hz 发送最新指令，0.3 秒无新指令发零，
    0.5 秒无 CAN 状态发零且停止发布伪新鲜 odom。
  - 本地修改 `scout_base_ros.hpp/.cpp`、`scout_base_node.cpp`：可停止运行循环，
    SIGINT/SIGTERM 只置标志，在主线程发零并销毁 ROS 对象后再关闭 context，修复退出崩溃。
  - 已编译并以 AGX_V2 协议连接真实 Scout Mini 差速底盘；架空低速控制和停车反馈通过。
- UGV SDK：官方 `agilexrobotics/ugv_sdk` main。
  - commit：`f2704eacdc90357078cd93ec60aae08bb4baab35`。
  - 网络限制下按 GitHub tree 获取 68 个构建所需文件，逐个核对 Git blob SHA-1；
    清单见 `src/ugv_sdk/SOURCE_MANIFEST.json`。未下载无关 PDF 和测试。
  - 本地修改 `src/async_port/async_can.cpp`：异步发送持有 CAN 帧副本直到完成，
    避免发送缓冲区引用已过期的调用方栈；发送失败时输出实际错误原因。
  - 两个源码目录用 `.upstream-revision` 标记版本，汇总见 `BASE_SOURCE_REVISIONS.txt`。
  - Asio 头文件来自 Ubuntu `libasio-dev` 1:1.18.1-1 官方 deb，解压于 `deps/asio`；
    `build.bash` 通过 `CPLUS_INCLUDE_PATH` 使用，不更改宿主机系统包。
- 视觉跟随源：`/home/frank/demo/src/visual_follow`，没有 Git 元数据。
  - 深度估计与迟滞控制模块从原工程复制；后续增加可配置的有效深度比例/像素门槛和质量统计。
    葡萄一步入口使用相机小块过滤值 20、中心区域有效比例 0.15、至少 20 像素；
    保持当前帧中央 40% 中值测距，缺深度不复用旧测距。画面和诊断话题增加停车原因及处理耗时。
  - ROS 节点移植为 rclpy；增加相机内参和 TF，按车体 XY 计算目标接近控制。
- 检测模型：`/home/frank/demo/yolov8n.pt`，已复制为本工程模型文件。
  - SHA256：`f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36`
- Nav2 初始参数来源：本机 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`。
  - 已改为差速、RGB-D 点云障碍输入、现场实测 footprint，以及独立指令出口。
  - YOLO 已用真实固定相机进行 GPU 推理；RTAB-Map 建图与实机导航尚未运行验收。
- 仿真：本机 `/home/frank/demo/src/scout`，保留源快照到 `reference/demo_scout`。
  - 迁移 19 个 world 文件及递归本地模型依赖；不依赖 GitHub 下载。
  - Piper 外观来自 `/home/frank/demo/src/piper/piper_description/piper_description_gazebo.xacro`。
  - 原始来源 SHA256 和场景转换记录：`src/scout_sim/ASSET_MANIFEST.json`。
  - 转换工具：`scripts/migrate_demo_assets.py`；仅在初次导入或恢复本工具的中断导入时运行。
  - 运行环境：Gazebo Fortress 6.18.0 / libsdformat 12.9.0 / ros_gz_bridge 0.244.26。
