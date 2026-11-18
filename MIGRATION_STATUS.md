# 迁移状态：2026-11-17

## 已完成并验证

- `/home/frank/scout` 已建立独立 ROS 2 Humble 工作空间。
- Orbbec SDK v1 兼容分支（相机包 1.5.15 / SDK 1.10.35）独立编译通过。
- `scout_nav_bringup`、`scout_visual_follow`、`scout_sim` 与 `orbbec_camera_msgs` 编译通过；合计 5 个包。
- 46 项离线测试通过：深度单位/无效深度、迟滞停车、倾斜相机坐标变换、陈旧图像拒绝、
  未测外参/轮廓检查、启动配置展开、Nav2 指令重映射、单来源限速/超时停止，
  以及全部迁移场景的 SDF、资源完整性、几何/保存位姿保留和仿真通信隔离。
- 已单独执行 ROS 运行测试：隔离在 DDS domain 199，所有控制话题重映射到测试路径；
  验证默认禁用、服务使能、新指令转发、超时停车并撤销使能。没有连接或控制真实底盘。
- `--show-args` 可解析；映射/导航的配置检查通过不代表 RTAB-Map 或底盘运行成功。
  RTAB-Map 缺失时仅对生成的启动描述做 mock 检查，没有模拟一次建图成功。

## 仿真迁移

- 原 `demo/src/scout` 已保留独立快照到 `reference/demo_scout`，ROS 1 包排除 colcon 构建。
- 19 个 world 文件（含重复版本）和 19 个模型依赖迁入 `scout_sim`，全部通过
  Fortress 原生 SDF 检查及实际加载/物理步进检查。
- 已修复移除 Classic `<state>` 时丢失保存位置/旋转的问题：先烘焙现存物体的保存位姿，
  再去掉运行快照，保持 link/visual/collision 局部变换。playpen 修正 26 个物体的位姿，
  其中 21 个涉及旋转；相关农业场景同步恢复。重新构建 `scout_sim`，并在 Gazebo 中核对
  两块路障及垃圾箱的实际位置/角度，记录在 `log/simulation/rotation_fix/native_poses.json`。
- 四轮底盘改用 Fortress DiffDrive + WheelSlip；提供轮式里程计、真实仿真位姿、关节状态和 TF。
- 隔离 domain 198 的空场景、domain 197 的原 playpen 场景运行测试通过：均带 RGB-D，
  实际前进约 0.54 m、左转约 0.62 rad，断开指令后停车。各收到 112 帧彩图、深度和点云，
  内参/分辨率/深度单位/帧名与 TF 检查通过；详细结果在 `log/simulation`。
- 图像使用 optical frame，Fortress 的 X-forward 原生点云单独标记为 `nav_camera_link`；
  不用错误的 optical 帧名掩盖点云坐标方向差异。倾角由 TF 处理。
- Piper 仅保留固定外观，五个行人场景转为静态版本；未移植机械臂控制或动态行人插件。
- 导航仿真默认固定旧场景的可移动道具，Scout 仍是动态模型；可通过
  `static_environment:=false` 恢复旧道具物理设置，该模式尚未完成持续运行验收。
- 使用 Cyclone DDS、独立 domain 42 和 `/scout_sim/cmd_vel`；不启动实机 CAN/相机/Piper 节点。
- GUI 窗口和硬件显卡渲染尚未人工验收；已验证无 GUI 软件渲染。
  尚未完成仿真 RTAB-Map + Nav2 建图/导航闭环。启动方法与限制见 `SIMULATION.md`。

## 待完成

1. GitHub DNS/连接恢复后获取并编译官方 Scout `humble` + `ugv_sdk`。
2. 在宿主机终端运行依赖脚本安装 RTAB-Map、libasio-dev 等；当前工具环境的 sudo
   受 no-new-privileges 限制，未执行系统安装。
3. 识别固定架子相机的序列号，并验证 RGB-D 注册、帧率和时间戳。
4. 锁紧相机铰链，实测 `base_link -> nav_camera_link` 的 xyz / roll / pitch / yaw。
5. 确认 Scout 专用 CAN 接口、实际完整 footprint，验证轮式里程计与失联停车。
6. 低速遥控建图、闭环及重定位；然后验证代价地图与短距离 Nav2 导航。
7. 独立安装/验证 YOLO 推理环境；最后接入目标识别导航任务及 Piper 到位交接。

## 硬件约定与当前边界

- Scout Mini 差速版；固定架子 Orbbec DaBai DCW 用于导航；另一台腕部相机属于抓取侧。
- 用户已确认铰链角度可调，但拧紧后行驶时固定。静态 TF 合适；每次调角度后必须更新外参。
- 没有把旧仿真的相机安装值或其他机器人项目的几何配置当作实测值。
- 当前工具环境未读到 Scout CAN 接口，也没有确认相机 USB 数据流。
- 未修改旧 `demo` 工程、Piper 工程、原 Orbbec checkout、Docker、CAN 或 `.bashrc`。
- 没有启动真实相机驱动、底盘驱动、SLAM 或 Nav2；没有向真实底盘发送运动指令。

操作入口：`README.md`；待实测配置：`config/robot.yaml`；来源：`SOURCE_REVISIONS.md`。
