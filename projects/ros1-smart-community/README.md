# ROS1 智慧社区巡检项目

此目录同步自 VMware 中的 ROS Noetic 项目。当前主要运行工作空间是 `/home/ros1/ce`，核心包是 `ceshi3`。本次同时保存了同一项目的历史工作空间 `zh1`、`zhihui1` 和 `zhihui2`，便于追溯。

## 2026-10-11 最新更新

当前源码直接读取自 `/home/ros1/ce/src/ceshi3`，覆盖 `robot.launch`、`nav.launch`、`patrol_with_vision.launch` 及其配置、地图、世界文件和视觉脚本。文件大小、执行权限和 SHA256 记录在 [本次同步清单](update-manifest-20261011.json)。

- `robot.launch` 加载 `15.world`。新增 Gazebo 同步插件 `src/traffic_light_sync.cpp`，用一个仿真时钟驱动四个灯箱，Gazebo GUI 和相机使用同一组静态灯色材质。
- 灯色周期：红灯 10、绿灯 15、黄灯 5 **仿真秒**。仿真暂停时灯色暂停；仿真速度低于实时，现实等待时间会相应增加。
- 巡检恢复为连续确认绿灯才放行；红灯、黄灯继续等待。当前路线没有前进 2 米操作，也没有过灯过渡点。
- 新增三张灯色纹理 `red.png`、`green.png`、`yellow.png`，与插件源码一同保存在 Git 仓库。

[下载本次更新包](https://github.com/50yan/1/releases/tag/ros1-project-update-20261011)。这份更新包是源码及本次插件备份的增量包；首次恢复需要先取得下面的 2026-10-10 完整备份，以补齐外部场景纹理、ONNX 和 OCR 权重，再覆盖本次更新包。既有完整备份和计时包保留为历史版本。

已有 `/home/ros1/ce` 环境时，先关闭仿真、导航和巡检节点，在下载目录执行：

```bash
sha256sum -c UPDATE_SHA256SUMS
tar -xzf ros1-project-update-20261011.tar.gz -C /home/ros1
source /opt/ros/noetic/setup.bash
cd /home/ros1/ce
catkin_make --pkg ceshi3 -j2 -l2
source /home/ros1/ce/devel/setup.bash
```

插件运行时从 `/home/ros1/ce/devel/lib/libceshi3_traffic_light_sync.so` 加载，因此恢复源码后必须重新编译。每个启动终端都应加载 `/home/ros1/ce/devel/setup.bash`。

## 源码与完整备份

Git 仓库提供源码、launch、地图、world、机器人描述、模型定义、配置和巡检文字报告。大体积文件（rosbag、ONNX、场景纹理、巡检原图、数据集、OCR 权重等）保存在 [完整备份 Release](https://github.com/50yan/1/releases/tag/ros1-project-20261010)。完整压缩包还包含各工作空间的 `build`、`devel` 及项目内历史备份。

`manifest.json` 对应完整快照中的文件、大小、SHA256 和保存位置。之后新增的计时改动、旧版恢复脚本和备份单独记录在 `timed-update-manifest.json`，并保存在 `ros1-project-timed-update.tar.gz`。该计时包是历史版本。当前仓库源码以 `update-manifest-20261011.json` 为准，使用连续确认绿灯放行和统一仿真时钟红绿灯。`repository_and_snapshot` 表示源码树与压缩包都有；`snapshot` 表示从 Release 下载完整压缩包恢复。压缩包保持 `/home/ros1` 下的原始相对路径。

为避免提交与项目无关的登录资料和可再生成缓存，备份排除了 `.claude`、Python 字节码缓存等；完整排除清单在 manifest 中。未同步操作系统、ROS 安装、Python 虚拟环境和全局 ROS 日志。依赖版本记录在 `environment.json`。

## 目录对应关系

| 仓库/压缩包路径 | 虚拟机原始路径 | 用途 |
| --- | --- | --- |
| `ce/` | `/home/ros1/ce` | 当前建图、导航、巡检、视觉识别及数据 |
| `building_editor_models/` | `/home/ros1/building_editor_models` | 建筑与墙壁模型 |
| `model_editor_models/` | `/home/ros1/model_editor_models` | 人物、车牌、红绿灯立牌及纹理 |
| `桌面/` | `/home/ros1/桌面` | ONNX、场景材料、复赛资料及项目笔记 |
| `zh1/`、`zhihui1/`、`zhihui2/` | 同名目录 | 同一项目的其他工作空间 |
| `camera/`、`备份/` | 同名目录 | 相机样例及历史代码/场景 |
| `.paddleocr/` | `/home/ros1/.paddleocr` | PaddleOCR 字符识别模型 |
| `.rviz/` | `/home/ros1/.rviz` | RViz 设置 |

## 恢复到 ROS 虚拟机

从完整备份 Release 下载所有 `ros1-project-full.tar.gz.part001` 至 `part011` 分卷、`SHA256SUMS` 和历史 `ros1-project-timed-update.tar.gz`，再从 2026-10-11 更新 Release 下载 `ros1-project-update-20261011.tar.gz`、`update-manifest-20261011.json` 和 `UPDATE_SHA256SUMS` 到同一目录，然后在 Linux 执行：

```bash
sha256sum --ignore-missing -c SHA256SUMS
cat ros1-project-full.tar.gz.part[0-9][0-9][0-9] > ros1-project-full.tar.gz
sha256sum --ignore-missing -c SHA256SUMS
mkdir -p ~/ros1_project_restore
tar -xzf ros1-project-full.tar.gz -C ~/ros1_project_restore
tar -xzf ros1-project-timed-update.tar.gz -C ~/ros1_project_restore
sha256sum -c UPDATE_SHA256SUMS
tar -xzf ros1-project-update-20261011.tar.gz -C ~/ros1_project_restore
```

先在临时目录查看文件，再将需要的目录恢复到 `/home/ros1` 对应位置。源码中有 `/home/ros1` 的绝对路径，因此建议沿用用户名及目录；换用户名时需调整 world 的材质路径、launch 和 Python 中的模型/结果路径。复制 Linux 文件时使用 `cp -a` 保留执行权限及链接。将新版本文件恢复到已运行的环境前应关闭对应节点。

`build` 和 `devel` 是备份资料；在新环境中应重新编译工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ros1/ce
catkin_make
source /home/ros1/ce/devel/setup.bash
```

## 当前巡检

实际运行的是 `ce/src/ceshi3/worlds/15.world`。Gazebo、导航和巡检在三个独立终端依次启动：

```bash
# 终端一
roslaunch ceshi3 robot.launch
# 终端二
roslaunch ceshi3 nav.launch
# 终端三
roslaunch ceshi3 patrol_with_vision.launch
```

巡检包含 11 个点：第 1、7 点判断红绿灯，第 2–6 点识别人物，第 8–11 点识别车牌。红灯、黄灯原地等待；绿灯至少连续确认 3 帧，且相机时间和现实经过时间都达到 0.4 秒后，自动导航到下一巡检点。新相机帧会持续更新，无需输入 `1/y/yes`。本次源码没有原先的红灯 4 秒/黄转红 5 秒倒计时、前进 2 米目标和过灯过渡点。

第 1 点固定裁剪为 `image[66:190, 270:750]`，第 7 点为 `image[5:160, 325:900]`。当前相机为 1280×720，按横向红、黄、绿灯排列判断 HSV 颜色与亮度。

当前权重：

- 人物：`/home/ros1/桌面/person_type_xz3_ros.onnx`，类别 `community`、`noncommunity`。
- 车牌：`/home/ros1/桌面/best_ros.onnx`。
- 红绿灯灯箱：`/home/ros1/桌面/best_ros2.onnx`，单类别 `traffic_light`；灯色由 OpenCV 判断。

结果保存在 `/home/ros1/ce/src/vision_data/shibie/<运行时间>/`，包括原图、检测图、红绿灯裁剪图和 JSON/CSV/TXT 报告。

此次同步对所有导出的 Python 文件进行了语法检查，对 launch、xacro、world 和 package.xml 进行了 XML 检查。此前插件部署完成 catkin 编译，并采集独立场景 639 帧和完整场景 861 帧；避开切换瞬间，灯色与仿真时钟一致。验证记录位于 `ce/patrol_backups/before_traffic_sync_20261011_125602/`。同步时没有重新运行 11 点完整路线。
