# 来源说明

`src/OrbbecSDK_ROS2` 为 Orbbec 官方源码快照，保留其 LICENSE、NOTICE 和源码版权信息。

`src/scout_nav_bringup/config/nav2_scout.yaml` 根据本机安装的 Navigation2 默认配置调整。
Navigation2 使用 Apache License 2.0；上游项目为
`https://github.com/ros-navigation/navigation2`。相关上游版权归各原作者所有。

`scout_visual_follow` 的原始跟随逻辑和模型来自用户已有 `/home/frank/demo` 工程。
模型来源和使用条件沿用原工程及 Ultralytics 上游说明；本目录中的新文件不改变其上游许可。

`scout_sim` 的 Scout 网格、world 和模型依赖来自用户本机 `demo/src/scout` 与
`~/.gazebo/models`。原 Scout 工程完整保留在 `reference/demo_scout`，含 Clearpath Robotics、
Weston Robot 等原作者的 BSD 声明；Piper 描述来源保留在 `reference/piper_description_source`。
模型文件中的原作者、model.config、LICENSE/说明文件均随资源复制。
个别资产的上游许可未在本机副本中完整注明，沿用原项目使用条件，不据此推定可任意再分发。
新启动代码和转换工具不改变原模型、场景或网格的权利归属。
