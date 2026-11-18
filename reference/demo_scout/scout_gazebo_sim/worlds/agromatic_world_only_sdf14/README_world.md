# Agromatic world-only SDF 1.4 package

这些 `.world` 文件已经把 `<sdf version>` 改为 `1.4`，可以放到任意位置运行。

推荐先使用：

- `agromatic_mondo_campo_pom_sdf14.world`：默认番茄田静态场景，对应原 `husky_sim.launch`。
- `agromatic_mondo_dynamic_sdf14.world`：带动态障碍物场景，对应原 `husky_sim_dynamic.launch`。
- `agromatic_mondo_campo_water_tank_sdf14.world`：带水箱模型场景，对应原 `husky_sim_water_tank.launch`。

运行前请把 `agromatic_models_for_gazebo.zip` 中的模型文件夹复制到 `~/.gazebo/models/`。

注意：原世界文件还引用 Gazebo 常见模型，例如 `oak_tree`、`pickup`、`dumpster`、`person_standing`、`nist_maze_wall_120`、`mud_box`、`fire_hydrant`、`ground_plane` 和 `sun`。其中 `tomato_0`、`tomato_fruit`、`water_tank` 已从原压缩包提取。其余常见模型如果本机 Gazebo 没有，需要另行安装或替换。
