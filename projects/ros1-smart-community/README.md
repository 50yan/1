# ROS1 智慧社区巡检项目

此目录同步自 VMware 中的 ROS Noetic 项目。当前主要运行工作空间是 `/home/ros1/ce`，核心包是 `ceshi3`。本次同时保存了同一项目的历史工作空间 `zh1`、`zhihui1` 和 `zhihui2`，便于追溯。

## 源码与完整备份

Git 仓库提供源码、launch、地图、world、机器人描述、模型定义、配置和巡检文字报告。大体积文件（rosbag、ONNX、场景纹理、巡检原图、数据集、OCR 权重等）保存在 [完整备份 Release](https://github.com/50yan/1/releases/tag/ros1-project-20261010)。完整压缩包还包含各工作空间的 `build`、`devel` 及项目内历史备份。

`manifest.json` 列出虚拟机原始目录中全部已备份文件、大小、SHA256、以及保存位置。`repository_and_snapshot` 表示源码树与压缩包都有；`snapshot` 表示从 Release 下载完整压缩包恢复。压缩包保持 `/home/ros1` 下的原始相对路径。

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

从 Release 下载 `ros1-project-full.tar.gz` 和 `SHA256SUMS` 到同一目录，然后在 Linux 执行：

```bash
sha256sum --ignore-missing -c SHA256SUMS
mkdir -p ~/ros1_project_restore
tar -xzf ros1-project-full.tar.gz -C ~/ros1_project_restore
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

启动前先检查 `ce/src/ceshi3/launch/robot.launch` 中的实际 world 配置。同步时虚拟机实际运行的是 `15.world`；本次同步没有修改场景选择。

Gazebo、导航和巡检分别在三个终端启动：

```bash
roslaunch ceshi3 robot.launch
roslaunch ceshi3 nav.launch
roslaunch ceshi3 patrol_with_vision.launch
```

巡检包含 11 个点：第 1、7 点判断红绿灯，第 2–6 点识别人物，第 8–11 点识别车牌。红灯、黄灯或无法确认时等待；持续取得新画面，连续确认绿灯后再前往下一点，无需 `1/y/yes` 输入。

第 1 点固定裁剪为 `image[66:190, 270:750]`，第 7 点为 `image[5:160, 325:900]`。当前相机为 1280×720，按横向红、黄、绿灯排列判断 HSV 颜色与亮度。

当前权重：

- 人物：`/home/ros1/桌面/person_type_xz3_ros.onnx`，类别 `community`、`noncommunity`。
- 车牌：`/home/ros1/桌面/best_ros.onnx`。
- 红绿灯灯箱：`/home/ros1/桌面/best_ros2.onnx`，单类别 `traffic_light`；灯色由 OpenCV 判断。

结果保存在 `/home/ros1/ce/src/vision_data/shibie/<运行时间>/`，包括原图、检测图、红绿灯裁剪图和 JSON/CSV/TXT 报告。

此次同步是文件备份，不代表重新运行完整巡检。上传前核对文件清单与哈希，红绿灯部署检查记录为编译通过、样例照片/纹理识别通过及 ROS 新相机帧检查通过。
