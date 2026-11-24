# Scout Mini 仿真与场景

已把本机 `/home/frank/demo/src/scout` 的模型和 world 迁入独立 ROS 2 包
`/home/frank/scout/src/scout_sim`。运行平台是 **ROS 2 Humble + Gazebo Fortress 6**。
本包不需要 Docker、ROS 1、`gazebo_ros`、CAN 或真实相机。

## 启动

```bash
bash ~/scout/scripts/build.bash
bash ~/scout/scripts/sim.bash
```

默认场景是原来的 `clearpath_playpen`，包含障碍物和原工程添加的黑色路线。
默认出生位置为 `x=0, y=-2`，避开原点前方紧邻的护栏。
导航模式默认 `static_environment:=true`：场景道具固定，Scout 本身仍参与物理仿真。
旧 playpen 的可移动垃圾箱/路锥等使用 Classic 接触参数；固定它们可避免这些参数干扰导航测试。
需要研究原有道具动力学时可设 `static_environment:=false`，该模式尚未完成运行验收。
需要 RViz 时：

```bash
bash ~/scout/scripts/sim.bash rviz:=true
```

切换果园场景或调整**仿真**相机俯角（正值朝下）：

```bash
bash ~/scout/scripts/sim.bash world:=orchard camera_pitch_deg:=20
```

所有场景的可选名称：

```bash
bash ~/scout/scripts/list_sim_worlds.bash
```

常用名称还有 `weston_robot_empty`、`cpr_orchard`、`orchard1`、
`agromatic_mondo_campo_pom_sdf14`、`agromatic_mondo_campo_water_tank_sdf14`。
子目录场景使用列表中完整名称，例如
`world:=agromatic_world_only_sdf14/campo_sdf14`。
部分农田场景原点位于种植区域，需要用 `x:=... y:=... yaw:=...` 选择通行位置；yaw 单位弧度。

没有图形界面时，可测试软件渲染，或只启动物理仿真：

```bash
bash ~/scout/scripts/sim.bash gui:=false software_rendering:=true
bash ~/scout/scripts/sim.bash gui:=false camera:=false
```

`camera:=false` 不启动渲染传感器；该模式可用于验证轮式底盘与里程计，不提供深度图。
GUI 和硬件加速需要宿主机图形会话及可用显卡驱动，不能用纯物理测试代替图像验收。
Ctrl+C 结束本次仿真。

## 话题与仿真控制

每个用于查看或控制仿真的新终端都先加载：

```bash
source ~/scout/scripts/sim_env.bash
```

此脚本选择 DDS domain 42、本机通信、Cyclone DDS 和独立 Gazebo partition。
真实工程的终端继续使用 `ros_env.bash`。仿真与实机不要共用 DDS domain。

| ROS 2 话题 | 用途 |
| --- | --- |
| `/clock` | 仿真时间 |
| `/scout/odom` | 四轮差速插件计算的轮式里程计 |
| `/scout_sim/ground_truth` | 仿真器实际位姿，供测试比较；不替代导航的轮式里程计 |
| `/joint_states`、`/tf`、`/tf_static` | 四个轮子、车体、支架、固定相机和被动 Piper 的变换 |
| `/nav_camera/color/image_raw` | 640×360 RGB 图像 |
| `/nav_camera/depth/image_raw` | 同视角注册深度，`32FC1`，单位米 |
| `/nav_camera/color/camera_info` | 相机内参 |
| `/nav_camera/depth/points` | 深度点云 |
| `/scout_sim/cmd_vel` | 仿真指令入口，`geometry_msgs/msg/Twist` |

图像和点云桥接采用 Reliable 发布；读取大消息时建议也选择 Reliable，
RViz 配置已采用该设置。纯软件渲染复杂场景时仿真可能明显慢于实时，属于性能限制。
图像/内参的 frame 是 `nav_camera_color_optical_frame`；Fortress 原生点云使用
X 前、Y 左、Z 上，因此点云 frame 为 `nav_camera_link`，通过 TF 处理安装俯角。
不能只把该点云的帧名改为 optical frame 而不旋转坐标。

例如以 10 Hz 低速前进，Ctrl+C 后在约 0.5 秒内触发仿真指令超时停车：

```bash
source ~/scout/scripts/sim_env.bash
ros2 topic pub -r 10 /scout_sim/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

主动停车：

```bash
ros2 topic pub --once /scout_sim/cmd_vel geometry_msgs/msg/Twist '{}'
```

仿真指令限幅为 0.3 m/s 和 0.6 rad/s，Y 方向速度不执行。
本包不会把命令发送到实机入口 `/scout/cmd_vel`。
数据话题名称与导航侧约定一致，但导航节点接入时仍需设置 `use_sim_time:=true`，
使用独立的仿真外参/轮廓配置，并把最终速度接到上述仿真入口。

## RGB-D 闭环与葡萄跟随切换

退出同一仿真域中的旧任务后，启动 `bash ~/scout/scripts/sim_mapping.bash gui:=true rviz:=true grapes:=true`。
`gui` 和 `rviz` 分别打开 Gazebo 和 RViz 窗口，默认都关闭。在启动终端按 `Ctrl+C` 结束整套建图仿真；在监听终端按只结束监听。
去掉 `grapes:=true` 可测无目标巡航，追加 `vision:=false` 可关闭识别，`wander:=false` 可关闭闭环后的游荡。
如果 RTAB-Map 是本地解包安装而非系统安装，可在启动前将 `SCOUT_RTABMAP_PREFIX` 设为其 Humble 安装前缀。

任务先沿 `mapping_course` 固定路线巡航。跑完路线且收到起点附近的视觉闭环时报告 `loop_closed`，然后在 `/map` 已知空闲区自主选短途目标。未知区域和障碍留出车体间距，没有可行路段时报告 `wander_blocked` 并停车。两圈无闭环报告 `loop_not_confirmed`。
连续三帧识别到同一目标且有有效深度后停车交接，随后跟随；出现其他葡萄时保持当前锁定。目标丢失的第一帧就停车，连续三帧无效则释放。剩余目标同样需要连续三帧确认；无目标则回到建图航点或游荡。跟随期间 RTAB-Map 不暂停。

```bash
source ~/scout/scripts/sim_env.bash
ros2 topic echo /scout_sim/mapping_state std_msgs/msg/String --field data
```

另开终端可添加第二串或移除照片靶。下面的位置靠近初始靶，是否入镜取决于小车朝向；`add` 的参数为名称、Gazebo 世界坐标 x/y（m）、朝向 yaw（rad，原始靶朝向为 0）：

```bash
bash ~/scout/scripts/sim_grape.bash add grape_second 3.02 2.95 0
bash ~/scout/scripts/sim_grape.bash remove grape_fixture
bash ~/scout/scripts/sim_grape.bash remove grape_second
```

观察 `detected_ids`（检测框 ID）和 `selected_id`（当前锁定），以及 `target_lost`、`stopping_before_wander`、`wandering` 的状态变化。
更新 DDS 配置后先退出旧仿真，重新启动，并在查询终端重新 `source ~/scout/scripts/sim_env.bash`。域 42 的自动参与者索引现为 0–63；需要另一套隔离仿真时，在各终端统一 `export SCOUT_SIM_DOMAIN=194` 后再加载环境。
同一 ROS 域只允许一套仿真。启动若提示 `/clock` 已有发布者，用 `ros2 topic info /clock --verbose` 查看，并退出旧仿真；直接重置 Gazebo 时间也会结束建图任务，请重启建图脚本。

占据地图发布到 `/map`，闭环事件发布到 `/scout/rtabmap/info`；退出任务后 RTAB-Map 保存数据库。
带葡萄验收中，车在约 1.60 m 处停车；移除靶后恢复第 2 个航点，一圈结束确认闭环（RTAB-Map 节点 171 → 5）。
该次日志与地图快照保存在本机的 `log/simulation/acceptance_grape_final/`。
闭环后游荡的验收记录在 `log/simulation/wander_continuous/events.jsonl`：小车先后向三个不同的地图目标行驶。
`log/simulation/wander_trial1/events.jsonl` 记录了闭环后增删两串葡萄的验收：保持原 ID，移除锁定目标时先停车，再锁定仍在视野内的另一串，两串移除后继续游荡。
四轮仿真的轮式里程计会高估转角，此任务使用 `/scout_sim/mapping_odom` 的 Gazebo 理想里程计，
闭环约束仍来自仿真 RGB-D 图像；`/scout/odom` 可用于对比。葡萄靶使用实物模型照片纹理，由仓库里的 detection-v1 权重识别。
游荡使用占据地图做短途规划及停车检查。Nav2、动态避障、其他果园场景及实机建图仍待验收。

## 迁移范围与差异

- 共 19 个 world 文件（含原工程的重复场景），以及 19 个递归模型依赖已复制。
  不依赖原用户的 `~/.gazebo/models`，启动时不下载 Fuel 模型。
- 原始 ROS 1 Scout 工程完整副本位于 `/home/frank/scout/reference/demo_scout`，
  由 `COLCON_IGNORE` 排除构建；原 `/home/frank/demo` 未改动。
  转换场景、资源来源 SHA256 和逐场景差异见
  `/home/frank/scout/src/scout_sim/ASSET_MANIFEST.json`。
- 原始物体几何和黑色路线保留。对于含 Classic `<state>` 的场景，先将保存快照中
  现存物体的位置和旋转写入模型初始位姿，再移除运行快照；无快照的场景保留原始位姿。
  不把快照的世界坐标 link 位姿误写为模型局部位姿，也不恢复仅在旧快照中存在的已删除物体。
  Ogre 材质脚本转换为显式颜色/可用纹理，外观可能与 Classic 有细节差异。
  修复了 DAE 的隐式纹理路径和缺少的本地嵌套模型资源。
- `orchard1.world` 缺少 SDF 外层及结束标签，且四个模型重名；已修复，保留其五个完整模型。
- 五个包含行人的场景改为**静态行人版本**，输出文件带 `_static.sdf`。
  保留首个轨迹点位置；未移植 `libActorCollisionsPlugin.so`、行走动画和骨骼碰撞。
  原始轨迹仍在 reference 中，不应把这些场景用于验证动态行人避让。
- 四轮控制改用 Fortress 原生 DiffDrive，增加侧滑模型、ROS 桥接和超时停车。
  使用旧 xacro 的几何参数，车体/轮子碰撞采用简化体，惯量和摩擦用于基础仿真，
  不是实车动力学辨识结果。原轮网格半径约 0.08 m，而旧 xacro 声明的轮半径为 0.16 m；
  本版保留 0.16 m 仿真半径，并缩放轮子视觉使其与碰撞体一致。
- `base_link` 采用地面投影原点，旧车体中心框架保留为 `chassis_link`。
  相机保留旧安装位置，增加标准 optical frame；可配置固定俯角。
  这些仿真值不写入实机的 `/home/frank/scout/config/robot.yaml`。
- 保留支架和 Piper 外观，Piper 关节冻结为原模型零位，不加载机械臂控制器；
  Piper 部分只用于外观，不做机械臂碰撞/抓取仿真。

## 检查

基础迁移验收：5 个包编译通过，19 个场景均完成原生加载/步进检查。
空场景和原 playpen 场景均通过带 RGB-D 的前进、转弯、指令超时停车和 TF 检查，
各连续采集 112 帧 RGB、深度与点云。GUI/硬件显卡渲染尚未人工验收。

2026-11-17 修复保存位姿丢失后，`scout_sim` 已重新构建，46 项离线测试通过，
包括全部 19 个场景的原生 SDF 校验、保存位置/旋转与局部几何变换保留检查。
在 Gazebo 实际加载的 playpen 中核对了 `jersey_barrier_10`、`jersey_barrier_15`、
`Dumpster_0` 的位置和旋转；原生查询记录见 `log/simulation/rotation_fix/native_poses.json`，
逐场景恢复记录见 `log/simulation/saved_layout_repair.json`。已打开的仿真需关闭后重新启动才能加载修复。

```bash
bash ~/scout/scripts/test.bash

source ~/scout/scripts/sim_env.bash
export ROS_DOMAIN_ID=198 IGN_PARTITION=scout_smoke_198
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/scout/scripts/smoke_sim.py --world weston_robot_empty --no-camera
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/scout/scripts/smoke_sim.py --world clearpath_playpen
```

自动运行检查会启动并关闭仿真，验证实际车体位移、正向转弯、超时停车、TF、话题数据，
有相机时还检查分辨率、帧名与有效深度；`--data-only` 只检查数据与 TF，不声称验证运动。
日志、JSON 结果和成功采集的相机图像保存在
`/home/frank/scout/log/simulation`。离线测试包含全部 world 的原生 SDF 检查与资源完整性检查。
