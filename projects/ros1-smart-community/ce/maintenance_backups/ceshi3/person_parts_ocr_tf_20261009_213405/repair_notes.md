# 人物、车牌及 point_6 导航修复

已直接部署到 VMware 虚拟机 `/home/ros1/ce`，Windows 的 `F:\桌面\147\patrol_with_vision.py` 也已同步。

## 原因与结果

1. point_5 原图有三个人，模型输出三个全身框及中间人物的额外上半身框。额外框与全身框的 IoU 小于普通 NMS 阈值，因此没有被过滤。新增人物局部框去重：小框至少 90% 被更大、更高置信度的框覆盖，且大框面积至少为小框的 1.5 倍时，删除小框。此处理仅用于人物模型，不按巡检点预设人数。
2. 上次只删除了巡检脚本的默认车牌替换规则，独立 `ocr.launch` 的 `plate_ocr_ros.py` 仍保留了错误规则。本次两个入口都删除 `plate_aliases` 参数读取和号码替换逻辑。OCR 原文保留在 `raw_text`；`text` 仅规范大小写、空格和分隔符。例如 `苏W·GT88V` 中的 W 会保留。
3. `action state=4` 表示 move_base 的 ABORTED，不是 Python 语法错误或 OCR 识别错误。失败时尚未到第六点，也没有执行第六点 OCR。日志连续报告 `map→odom` TF 需要向未来外推，控制器不能转换全局路线；恢复旋转又被判断有碰撞风险，最后中止。AMCL 原本两次配置 `transform_tolerance`，最后的 0.1 秒覆盖前面的 0.2 秒；现改为唯一的 0.5 秒。
4. 导航中止后新增最多两次重试，调用 move_base 的代价地图清理服务，并等待两秒。激光仍会重新填充障碍物观测。单个点的所有尝试共用 30 分钟等待预算。若最终仍无法到达，保留失败状态和 move_base 原因，终止巡检，不伪造到达或拍照结果。

## 更改的文件与行号

以下均为部署后的虚拟机文件行号。

| 文件 | 行号 | 修改内容 |
| --- | --- | --- |
| `/home/ros1/ce/src/ceshi3/scripts/patrol_with_vision.py` | 113 | 新增 `suppress_person_parts`，删除全身框覆盖的重复局部框。 |
| 同上 | 208 | 人物模型执行上述局部框过滤。 |
| 同上 | 235 | 读取起始巡检点 `start_point`，并检查范围。 |
| 同上 | 238 | 导航失败最多重试两次，支持参数调整。 |
| 同上 | 348 | 输出实际 OCR 文本，不再应用车牌别名替换。 |
| 同上 | 494 | 报告说明改为 OCR 原文与格式规范化，删除“固定场景纠错”说明。 |
| 同上 | 532 | 新增 `navigate_to`，记录 move_base 原因、清理代价地图及有限重试。 |
| 同上 | 534 | 所有尝试共用 1800 秒，即 30 分钟。 |
| 同上 | 582 | 根据 `start_point` 跳过此前巡检点。 |
| `/home/ros1/ce/src/ceshi3/scripts/plate_ocr_ros.py` | 101 | 独立 OCR 节点直接使用识别文字，删除号码替换。 |
| `/home/ros1/ce/src/ceshi3/launch/patrol_with_vision.launch` | 3 | 新增 `start_point` 参数，默认 1。 |
| 同上 | 4 | 新增 `navigation_retries` 参数，默认 2。 |
| 同上 | 9 | 将两个参数传入巡检节点。 |
| `/home/ros1/ce/src/ceshi3/launch/amcl.launch` | 7 | `transform_tolerance` 统一为 0.5 秒，移除重复的 0.1 秒配置。 |
| `/home/ros1/ce/src/ceshi3/package.xml` | 80 | 添加调用导航代价地图清理服务需要的 `std_srvs` 运行依赖。 |

部署前比较了新旧 `PATROL_POINTS`，保持用户当前全部巡航坐标一致。

## 验证

- 真实保存图像回放：`20261009_205951` 的 point_5 为 community 3、noncommunity 0。
- `20261009_194128` 的 point_5 为 community 3、noncommunity 0；point_1 为 community 2、noncommunity 1。
- 通过完整车牌裁剪与独立 OCR 子进程流程回放 point_7，输出 `苏W·GT88V`，`raw_text` 同样是 `苏W·GT88V`，没有应用号码替换。
- 相邻两个人的框不会因包含过滤而被合并；更低置信度的大框不能删除更高置信度的小框。
- 导航模拟回调检查：ABORTED 后重试可接受 SUCCEEDED；连续三次 ABORTED 仍报告失败。
- Python 编译、`catkin_make` 和 `roslaunch --nodes ceshi3 patrol_with_vision.launch start_point:=6` 通过。
- 当前运行中的 AMCL 已更新为 0.5 秒；TF 抽样检查成功 20/20 次。当前位置到第六点可生成 73 个路线姿态。
- 实际发送第六点导航目标后，move_base 返回 `state=3, Goal reached`，耗时 10.4 秒。小车现停在 point_6。此验证没有重跑完整九点巡检。

## 使用方式

正常重新开始九点巡检时，分别在三个终端执行。每个终端都先加载环境。

终端一：

```bash
source /opt/ros/noetic/setup.bash
source ~/ce/devel/setup.bash
roslaunch ceshi3 robot.launch
```

终端二：

```bash
source /opt/ros/noetic/setup.bash
source ~/ce/devel/setup.bash
roslaunch ceshi3 nav.launch
```

终端三：

```bash
source /opt/ros/noetic/setup.bash
source ~/ce/devel/setup.bash
roslaunch ceshi3 patrol_with_vision.launch
```

已有 robot.launch 和 nav.launch 正常运行时，不要重复启动它们。新启动巡检进程即可读入更新的代码。下次重启 nav.launch 时会自动使用更新的 AMCL 参数。

当前只想从第六点继续测试，可在巡检终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ce/devel/setup.bash
roslaunch ceshi3 patrol_with_vision.launch start_point:=6
```

从第六点启动会新建一份报告，仅记录本次第六至第九点，不合并此前第一至第五点的旧报告。需要完整九点结果时使用默认启动命令。

巡检流程不需要额外启动 `ocr.launch`。巡检脚本会自动调用独立 OCR 子进程。

报告位于 `/home/ros1/ce/src/vision_data/shibie/<本次时间>/reports/`。历史报告保持原样，不会用新算法覆盖旧记录。

## 备份

虚拟机修改前的备份：

`/home/ros1/ce/maintenance_backups/ceshi3/person_parts_ocr_tf_20261009_213405/`

Windows 原巡检脚本已备份在 `F:\桌面\147\maintenance_backups\personfix_<时间>\patrol_with_vision.py.before`。
