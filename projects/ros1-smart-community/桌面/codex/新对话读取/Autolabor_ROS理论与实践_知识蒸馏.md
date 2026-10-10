# Autolabor ROS 理论与实践知识蒸馏

> 来源：`/home/ros1/桌面/codex/Autolabor_ROS理论与实践_全教程.pdf`  
> 用途：新对话中读取本文件，即可快速恢复 Autolabor ROS 教程中对后续写项目有用的知识上下文。  
> 范围：已刻意跳过虚拟机、Ubuntu、ROS、VSCode 等安装步骤；保留 ROS 项目开发、通信、运行管理、仿真、导航、机器人平台、进阶机制等知识。

---

## 0. 使用约定

本教程主要面向 **ROS 1**，常见版本语境是 Noetic/Melodic/Kinetic。写项目时默认使用：

- 工作空间：`catkin_ws`
- 构建系统：`catkin_make` 或 catkin
- C++ 客户端库：`roscpp`
- Python 客户端库：`rospy`
- 常用包结构：

```text
catkin_ws/
  src/
    pkg_name/
      CMakeLists.txt
      package.xml
      src/
      scripts/
      launch/
      msg/
      srv/
      action/
      cfg/
      param/
      urdf/
      rviz/
      worlds/
      maps/
```

写项目时，优先遵循这些原则：

1. 连续数据流用 **Topic**。
2. 短请求/短响应用 **Service**。
3. 长任务、可反馈、可取消用 **Action**。
4. 配置、阈值、调参用 **Parameter Server** 或 `dynamic_reconfigure`。
5. 多节点启动和参数配置用 **launch 文件**。
6. 坐标关系统一放进 **TF/TF2 树**。
7. 机器人模型用 **URDF/xacro**。
8. 仿真用 **Gazebo + Rviz**。
9. 导航核心链路是：`map -> odom -> base_link/base_footprint -> sensor_frame`。

---

# 1. ROS 基础概念与架构

## 1.1 ROS 是什么

ROS 全称 Robot Operating System，机器人操作系统。它不是传统意义上的完整操作系统，而是一个适用于机器人的开源元操作系统/软件框架。

ROS 提供：

- 通信机制
- 工具链
- 软件包管理方式
- 硬件抽象
- 消息定义
- 可视化工具
- 仿真工具
- 社区生态

教程中总结为：

```text
ROS = Plumbing + Tools + Capabilities + Ecosystem
```

即：

- Plumbing：通信管道
- Tools：工具
- Capabilities：机器人能力组件
- Ecosystem：生态系统

## 1.2 ROS 设计目标

ROS 的核心设计目标：

- 代码复用
- 分布式
- 松耦合
- 精简
- 语言独立
- 易于测试
- 支持大型应用
- 丰富工具包
- 免费开源

项目开发时最重要的是：

- 一个功能通常封装为一个功能包。
- 一个运行单元通常是一个节点。
- 节点之间不直接硬编码互相调用，而是通过 ROS 标准通信接口连接。
- 节点可以运行在同一台主机，也可以分布在多台主机。

---

# 2. ROS 文件系统与常用命令

## 2.1 ROS 文件系统结构

常见结构：

```text
workspace/
  src/
    package/
      CMakeLists.txt
      package.xml
      src/
      scripts/
      launch/
      msg/
      srv/
      action/
      cfg/
      param/
```

功能包是 ROS 代码组织的基本单位。一个包通常包含：

- 源代码
- 脚本
- 消息定义
- 服务定义
- launch 文件
- 参数文件
- 机器人模型
- 配置文件

## 2.2 创建功能包

```bash
catkin_create_pkg 包名 依赖包...
```

例：

```bash
catkin_create_pkg demo01 roscpp rospy std_msgs
```

## 2.3 安装/删除 ROS 包

```bash
sudo apt install ros-noetic-xxx
sudo apt purge ros-noetic-xxx
```

## 2.4 查询相关命令

```bash
rospack list              # 列出所有功能包
rospack find 包名          # 查找功能包路径
roscd 包名                 # 进入功能包目录
rosls 包名                 # 列出功能包文件
apt search xxx            # 搜索 apt 软件包
```

## 2.5 修改文件

```bash
rosed 包名 文件名
```

例：

```bash
rosed turtlesim Color.msg
```

## 2.6 执行相关命令

```bash
roscore
roscore -p 11312
rosrun 包名 可执行文件名
roslaunch 包名 launch文件名
```

`roscore` 会启动：

- ROS Master
- 参数服务器
- `rosout` 日志节点

---

# 3. ROS 计算图与通信机制

ROS 是以节点为单位运行的分布式框架。机器人系统通常由多个节点组成，例如：

- 摄像头节点
- 雷达节点
- 底盘控制节点
- 定位节点
- 路径规划节点
- 可视化节点

节点之间通过 ROS 通信机制交换数据。

ROS 基本通信机制：

1. 话题通信：发布订阅模型
2. 服务通信：请求响应模型
3. 参数服务器：参数共享模型

进阶通信机制：

4. Action 通信：长任务目标、反馈、结果、取消

---

# 4. 话题通信 Topic

## 4.1 使用场景

话题适合：

- 连续数据
- 高频数据
- 异步通信
- 一对多、多对多

典型例子：

- `/cmd_vel`：速度指令
- `/odom`：里程计
- `/scan`：激光雷达
- `/camera/image_raw`：图像
- `/tf`：坐标变换

## 4.2 理论模型

话题通信角色：

- Publisher：发布者
- Subscriber：订阅者
- Topic：话题
- Message：消息
- ROS Master：负责注册和发现

流程：

1. Publisher 向 Master 注册话题。
2. Subscriber 向 Master 查询话题。
3. Master 返回通信信息。
4. Publisher 与 Subscriber 建立连接。
5. 后续数据直接传输。

## 4.3 C++ 发布者模板

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
    ros::Rate rate(1);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "hello";
        pub.publish(msg);
        ROS_INFO("send: %s", msg.data.c_str());
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```

## 4.4 C++ 订阅者模板

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void cb(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("recv: %s", msg->data.c_str());
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter", 10, cb);
    ros::spin();
    return 0;
}
```

## 4.5 Python 发布者模板

```python
#! /usr/bin/env python
import rospy
from std_msgs.msg import String

rospy.init_node("talker_p")
pub = rospy.Publisher("chatter", String, queue_size=10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    msg = String()
    msg.data = "hello"
    pub.publish(msg)
    rospy.loginfo("send: %s", msg.data)
    rate.sleep()
```

## 4.6 Python 订阅者模板

```python
#! /usr/bin/env python
import rospy
from std_msgs.msg import String

def cb(msg):
    rospy.loginfo("recv: %s", msg.data)

rospy.init_node("listener_p")
sub = rospy.Subscriber("chatter", String, cb, queue_size=10)
rospy.spin()
```

---

# 5. 自定义 msg

## 5.1 文件位置

```text
pkg_name/
  msg/
    Person.msg
```

## 5.2 msg 示例

```text
string name
uint16 age
float64 height
```

## 5.3 package.xml 配置

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

如果包里还有 C++/Python/std_msgs：

```xml
<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>message_generation</build_depend>

<exec_depend>roscpp</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>message_runtime</exec_depend>
```

## 5.4 CMakeLists.txt 配置

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Person.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
```

## 5.5 C++ 使用自定义消息

```cpp
#include "pkg_name/Person.h"

pkg_name::Person p;
p.name = "robot";
p.age = 1;
p.height = 1.2;
```

CMake 中要添加消息生成依赖：

```cmake
add_executable(person_talker src/person_talker.cpp)
add_dependencies(person_talker ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(person_talker ${catkin_LIBRARIES})
```

## 5.6 Python 使用自定义消息

```python
from pkg_name.msg import Person

p = Person()
p.name = "robot"
p.age = 1
p.height = 1.2
```

---

# 6. 服务通信 Service

## 6.1 使用场景

服务适合：

- 一次请求，一次响应
- 需要明确返回结果
- 不适合持续高频传输

典型例子：

- 生成一只乌龟
- 清空轨迹
- 重置仿真
- 计算两个数之和
- 触发一次保存地图

## 6.2 理论模型

服务通信角色：

- Server：服务端
- Client：客户端
- Service：服务名称
- Request：请求
- Response：响应

流程：

1. Server 向 Master 注册服务。
2. Client 向 Master 查询服务。
3. Client 与 Server 建立连接。
4. Client 发送请求。
5. Server 处理请求并返回响应。

## 6.3 自定义 srv 文件

位置：

```text
pkg_name/
  srv/
    AddInts.srv
```

内容：

```text
int32 num1
int32 num2
---
int32 sum
```

`---` 上面是请求，下面是响应。

## 6.4 CMakeLists.txt 配置

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  AddInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
```

## 6.5 C++ 服务端模板

```cpp
#include "ros/ros.h"
#include "pkg_name/AddInts.h"

bool cb(pkg_name::AddInts::Request& req,
        pkg_name::AddInts::Response& resp) {
    resp.sum = req.num1 + req.num2;
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "addints_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("AddInts", cb);
    ros::spin();
    return 0;
}
```

## 6.6 C++ 客户端模板

```cpp
#include "ros/ros.h"
#include "pkg_name/AddInts.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "addints_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<pkg_name::AddInts>("AddInts");
    client.waitForExistence();

    pkg_name::AddInts srv;
    srv.request.num1 = 3;
    srv.request.num2 = 4;

    if (client.call(srv)) {
        ROS_INFO("sum = %d", srv.response.sum);
    } else {
        ROS_ERROR("service call failed");
    }
    return 0;
}
```

## 6.7 Python 服务端模板

```python
#! /usr/bin/env python
import rospy
from pkg_name.srv import AddInts, AddIntsResponse

def cb(req):
    return AddIntsResponse(req.num1 + req.num2)

rospy.init_node("addints_server_p")
server = rospy.Service("AddInts", AddInts, cb)
rospy.spin()
```

## 6.8 Python 客户端模板

```python
#! /usr/bin/env python
import rospy
from pkg_name.srv import AddInts

rospy.init_node("addints_client_p")
client = rospy.ServiceProxy("AddInts", AddInts)
client.wait_for_service()
resp = client(3, 4)
rospy.loginfo("sum = %d", resp.sum)
```

---

# 7. 参数服务器 Parameter Server

## 7.1 使用场景

参数服务器适合共享配置，不适合高频数据。

典型参数：

- 机器人名称
- 最大速度
- PID 参数
- 雷达 frame
- 地图路径
- 背景颜色
- 算法阈值

## 7.2 C++ 参数操作

```cpp
ros::NodeHandle nh;

nh.setParam("robot_name", "autobot");
nh.setParam("speed", 1.5);
nh.setParam("enabled", true);

std::string name;
double speed;
bool enabled;

nh.getParam("robot_name", name);
nh.getParam("speed", speed);
nh.getParam("enabled", enabled);

int count = nh.param("count", 10);

nh.deleteParam("speed");

ros::param::set("global_param", 123);
ros::param::get("global_param", count);
ros::param::del("global_param");
```

## 7.3 Python 参数操作

```python
import rospy

rospy.set_param("robot_name", "autobot")
rospy.set_param("speed", 1.5)
rospy.set_param("enabled", True)

name = rospy.get_param("robot_name", "default_name")
speed = rospy.get_param("speed")
exists = rospy.has_param("enabled")
names = rospy.get_param_names()
found_key = rospy.search_param("robot_name")

if rospy.has_param("speed"):
    rospy.delete_param("speed")
```

## 7.4 rosparam 命令

```bash
rosparam list
rosparam set /name value
rosparam get /name
rosparam delete /name
rosparam load config.yaml
rosparam dump config.yaml
```

YAML 示例：

```yaml
speed: 1.0
robot_name: robot1
controllers:
  p: 1.0
  i: 0.0
  d: 0.1
```

---

# 8. 常用 ROS 调试命令

## 8.1 节点 rosnode

```bash
rosnode list
rosnode info /node_name
rosnode ping /node_name
rosnode kill /node_name
rosnode cleanup
```

## 8.2 话题 rostopic

```bash
rostopic list
rostopic list -v
rostopic info /topic
rostopic type /topic
rostopic echo /topic
rostopic hz /topic
rostopic bw /topic
rostopic find 消息类型
rostopic pub /topic msg_type "data"
```

速度发布示例：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
"linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

循环发布：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```

## 8.3 消息 rosmsg

```bash
rosmsg list
rosmsg show std_msgs/String
rosmsg show geometry_msgs/Twist
rosmsg info 消息类型
rosmsg package 包名
rosmsg packages
```

## 8.4 服务 rosservice

```bash
rosservice list
rosservice info /service
rosservice type /service
rosservice args /service
rosservice call /service "args"
rosservice find srv类型
rosservice uri /service
```

示例：

```bash
rosservice call /spawn "x: 1.0
y: 2.0
theta: 0.0
name: 'turtle2'"
```

## 8.5 服务定义 rossrv

```bash
rossrv list
rossrv show turtlesim/Spawn
rossrv info srv类型
rossrv package 包名
rossrv packages
```

---

# 9. ROS 常用 API

## 9.1 初始化

C++：

```cpp
ros::init(argc, argv, "node_name");
ros::NodeHandle nh;
```

Python：

```python
rospy.init_node("node_name")
```

## 9.2 发布/订阅对象

C++：

```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("topic", 10);
ros::Subscriber sub = nh.subscribe<std_msgs::String>("topic", 10, cb);
```

Python：

```python
pub = rospy.Publisher("topic", String, queue_size=10)
sub = rospy.Subscriber("topic", String, cb, queue_size=10)
```

## 9.3 服务对象

C++：

```cpp
ros::ServiceServer server = nh.advertiseService("service", cb);
ros::ServiceClient client = nh.serviceClient<pkg_name::SrvType>("service");
```

Python：

```python
server = rospy.Service("service", SrvType, cb)
client = rospy.ServiceProxy("service", SrvType)
```

## 9.4 回旋函数

```cpp
ros::spin();      // 持续处理回调
ros::spinOnce();  // 处理一次回调
```

```python
rospy.spin()
```

使用原则：

- 订阅者/服务端一般用 `spin()`。
- 主循环中需要周期性处理逻辑时用 `spinOnce()` + `Rate`。

## 9.5 频率控制

C++：

```cpp
ros::Rate rate(10);
while (ros::ok()) {
    rate.sleep();
}
```

Python：

```python
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
```

## 9.6 时间与持续时间

C++：

```cpp
ros::Time now = ros::Time::now();
double seconds = now.toSec();
ros::Duration d(3.0);
d.sleep();
```

Python：

```python
now = rospy.Time.now()
seconds = now.to_sec()
d = rospy.Duration(3.0)
d.sleep()
```

## 9.7 日志

C++：

```cpp
ROS_DEBUG("debug");
ROS_INFO("info");
ROS_WARN("warn");
ROS_ERROR("error");
ROS_FATAL("fatal");
```

Python：

```python
rospy.logdebug("debug")
rospy.loginfo("info")
rospy.logwarn("warn")
rospy.logerr("error")
rospy.logfatal("fatal")
```

---

# 10. Python 模块导入

教程中提到一种脚本导入方式：手动把脚本目录加入 `sys.path`。

```python
import os
import sys

path = os.path.abspath(".")
sys.path.insert(0, path + "/src/pkg_name/scripts")

import tools
```

实用建议：

- 教程式小项目可以这样做。
- 正式项目更推荐把 Python 代码整理成包结构，例如：

```text
pkg_name/
  scripts/
    node.py
  src/
    pkg_name/
      __init__.py
      tools.py
```

然后在脚本中：

```python
from pkg_name import tools
```

Python 脚本要有执行权限：

```bash
chmod +x scripts/*.py
```

CMake 安装 Python 脚本：

```cmake
catkin_install_python(PROGRAMS
  scripts/talker.py
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

---

# 11. C++ 头文件与源文件组织

推荐结构：

```text
pkg_name/
  include/
    pkg_name/
      helper.h
  src/
    helper.cpp
    main_node.cpp
```

CMake：

```cmake
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_node src/main_node.cpp src/helper.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})
```

头文件引用：

```cpp
#include "pkg_name/helper.h"
```

---

# 12. launch 文件与运行管理

## 12.1 基本 launch

```xml
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="sim" output="screen"/>
  <node pkg="pkg_name" type="my_node" name="my_node" output="screen"/>
</launch>
```

运行：

```bash
roslaunch pkg_name file.launch
```

## 12.2 node 标签常用属性

```xml
<node
  pkg="package_name"
  type="executable_name"
  name="runtime_node_name"
  args="arg1 arg2"
  output="screen"
  respawn="true"
  respawn_delay="2"
  required="false"
/>
```

说明：

- `pkg`：包名
- `type`：可执行文件名
- `name`：运行时节点名
- `args`：传递参数
- `output="screen"`：日志输出到屏幕
- `respawn="true"`：节点退出后自动重启
- `required="true"`：该节点退出则整个 launch 退出

注意：`roslaunch` 不保证节点启动顺序；如果依赖服务，代码中应使用 `waitForService`。

## 12.3 include 标签

```xml
<include file="$(find other_pkg)/launch/other.launch"/>
```

带参数：

```xml
<include file="$(find other_pkg)/launch/other.launch">
  <arg name="robot_name" value="robot1"/>
</include>
```

## 12.4 remap 标签

```xml
<node pkg="pkg_name" type="node_exec" name="node_name">
  <remap from="old_topic" to="new_topic"/>
</node>
```

命令行重映射：

```bash
rosrun pkg node old_topic:=new_topic
```

## 12.5 param 与 rosparam

单个参数：

```xml
<param name="speed" value="1.0" type="double"/>
<param name="robot_name" value="robot1" type="str"/>
```

加载 YAML：

```xml
<rosparam command="load" file="$(find pkg_name)/cfg/config.yaml"/>
```

节点私有参数：

```xml
<node pkg="pkg_name" type="node_exec" name="node_name">
  <param name="speed" value="1.0"/>
</node>
```

代码读取私有参数：

```python
rate = rospy.get_param("~rate", 10)
```

```cpp
ros::NodeHandle private_nh("~");
private_nh.getParam("rate", rate);
```

## 12.6 group 命名空间

```xml
<launch>
  <group ns="robot1">
    <node pkg="pkg_name" type="driver" name="driver"/>
  </group>

  <group ns="robot2">
    <node pkg="pkg_name" type="driver" name="driver"/>
  </group>
</launch>
```

适合多机器人或多套同名节点。

## 12.7 arg 参数

```xml
<launch>
  <arg name="robot_name" default="robot1"/>
  <param name="name" value="$(arg robot_name)"/>
</launch>
```

运行时覆盖：

```bash
roslaunch pkg_name file.launch robot_name:=robot2
```

---

# 13. 名称、命名空间与重映射

## 13.1 节点名称设置

```bash
rosrun pkg node __name:=new_node_name
```

## 13.2 命名空间设置

```bash
rosrun pkg node __ns:=robot1
```

组合：

```bash
rosrun pkg node __ns:=robot1 __name:=driver
```

## 13.3 名称类型

- 全局名称：以 `/` 开头，如 `/chatter`，不受命名空间影响。
- 相对名称：无 `/`，如 `chatter`，会放到当前命名空间下。
- 私有名称：以 `~` 开头，如 `~rate`，属于节点私有命名空间。

C++：

```cpp
ros::Publisher pub1 = nh.advertise<std_msgs::String>("/global_topic", 10);
ros::Publisher pub2 = nh.advertise<std_msgs::String>("relative_topic", 10);
ros::NodeHandle private_nh("~");
private_nh.getParam("rate", rate);
```

Python：

```python
pub1 = rospy.Publisher("/global_topic", String, queue_size=10)
pub2 = rospy.Publisher("relative_topic", String, queue_size=10)
rate = rospy.get_param("~rate", 10)
```

---

# 14. 工作空间覆盖

多个工作空间可以被 source，ROS 会按环境变量顺序查找包。

风险：不同工作空间中出现同名包，会导致实际运行的包不是你以为的那个。

检查：

```bash
echo $ROS_PACKAGE_PATH
rospack find 包名
roscd 包名
```

建议：除非明确需要覆盖，否则不要在多个已 source 的工作空间中使用同名包。

---

# 15. ROS 分布式通信

ROS 1 支持多机通信。

前提：

- 多台机器网络互通。
- 主机名或 IP 能解析。
- 时间同步较好。
- 所有机器 ROS 版本和消息定义兼容。

主机：

```bash
export ROS_MASTER_URI=http://master_ip:11311
export ROS_HOSTNAME=master_ip
# 或 export ROS_IP=master_ip
roscore
```

从机：

```bash
export ROS_MASTER_URI=http://master_ip:11311
export ROS_HOSTNAME=this_machine_ip
# 或 export ROS_IP=this_machine_ip
```

检查：

```bash
ping other_machine
rosnode list
rostopic list
rostopic echo /topic
```

---

# 16. TF / TF2 坐标变换

## 16.1 作用

TF 用于维护机器人不同坐标系之间的关系。

常见坐标：

```text
map
odom
base_link
base_footprint
laser
camera_link
camera_depth_frame
wheel_link
```

导航中关键树：

```text
map -> odom -> base_link/base_footprint -> sensor_frame
```

## 16.2 常见消息

```bash
rosmsg info geometry_msgs/TransformStamped
rosmsg info geometry_msgs/PointStamped
rosmsg info tf/tfMessage
```

`TransformStamped` 关键字段：

```text
header.frame_id      父坐标系
child_frame_id       子坐标系
transform.translation 平移
transform.rotation    四元数旋转
```

## 16.3 静态坐标变换

用于固定不变的坐标关系，例如：

```text
base_link -> laser
base_link -> camera_link
```

命令：

```bash
rosrun tf2_ros static_transform_publisher x y z roll pitch yaw parent_frame child_frame
```

例：

```bash
rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 base_link laser
```

C++：

```cpp
tf2_ros::StaticTransformBroadcaster broadcaster;
geometry_msgs::TransformStamped ts;

ts.header.stamp = ros::Time::now();
ts.header.frame_id = "base_link";
ts.child_frame_id = "laser";
ts.transform.translation.x = 0.2;
ts.transform.translation.y = 0.0;
ts.transform.translation.z = 0.5;

tf2::Quaternion q;
q.setRPY(0, 0, 0);
ts.transform.rotation.x = q.getX();
ts.transform.rotation.y = q.getY();
ts.transform.rotation.z = q.getZ();
ts.transform.rotation.w = q.getW();

broadcaster.sendTransform(ts);
```

Python：

```python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

br = tf2_ros.StaticTransformBroadcaster()
t = TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "laser"
t.transform.translation.x = 0.2
t.transform.translation.y = 0.0
t.transform.translation.z = 0.5

q = tf.transformations.quaternion_from_euler(0, 0, 0)
t.transform.rotation.x = q[0]
t.transform.rotation.y = q[1]
t.transform.rotation.z = q[2]
t.transform.rotation.w = q[3]

br.sendTransform(t)
```

## 16.4 动态坐标变换

用于随时间变化的坐标关系，例如：

```text
odom -> base_link
map -> odom
world -> turtle1
```

C++：

```cpp
tf2_ros::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped tfs;

tfs.header.stamp = ros::Time::now();
tfs.header.frame_id = "world";
tfs.child_frame_id = "base_link";
tfs.transform.translation.x = pose->x;
tfs.transform.translation.y = pose->y;
tfs.transform.translation.z = 0.0;

tf2::Quaternion q;
q.setRPY(0, 0, pose->theta);
tfs.transform.rotation.x = q.getX();
tfs.transform.rotation.y = q.getY();
tfs.transform.rotation.z = q.getZ();
tfs.transform.rotation.w = q.getW();

broadcaster.sendTransform(tfs);
```

Python：

```python
br = tf2_ros.TransformBroadcaster()
t = TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "world"
t.child_frame_id = "base_link"
t.transform.translation.x = pose.x
t.transform.translation.y = pose.y
t.transform.translation.z = 0.0

q = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
t.transform.rotation.x = q[0]
t.transform.rotation.y = q[1]
t.transform.rotation.z = q[2]
t.transform.rotation.w = q[3]

br.sendTransform(t)
```

## 16.5 坐标监听与转换

C++：

```cpp
tf2_ros::Buffer buffer;
tf2_ros::TransformListener listener(buffer);

geometry_msgs::PointStamped src;
src.header.frame_id = "laser";
src.header.stamp = ros::Time(0);
src.point.x = 1.0;
src.point.y = 2.0;
src.point.z = 0.0;

geometry_msgs::PointStamped dst;
dst = buffer.transform(src, "base_link");
```

需要：

```cpp
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
```

Python：

```python
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)

src = PointStamped()
src.header.frame_id = "laser"
src.header.stamp = rospy.Time(0)
src.point.x = 1.0
src.point.y = 2.0
src.point.z = 0.0

dst = buffer.transform(src, "base_link", rospy.Duration(1.0))
```

查询坐标关系：

```python
trans = buffer.lookup_transform("target_frame", "source_frame", rospy.Time(0), rospy.Duration(1.0))
```

注意：`lookup_transform(target, source, time)`，目标坐标系在前，源坐标系在后。

## 16.6 TF 调试命令

```bash
rosrun tf2_tools view_frames.py
evince frames.pdf
rosrun tf tf_echo parent child
rosrun rqt_tf_tree rqt_tf_tree
```

---

# 17. rosbag

## 17.1 作用

rosbag 用于记录和回放 ROS 话题数据，适合调试、复现实验、离线分析。

## 17.2 命令行

记录全部话题：

```bash
rosbag record -a -O output.bag
```

记录指定话题：

```bash
rosbag record /cmd_vel /odom /scan -O nav_debug.bag
```

查看：

```bash
rosbag info nav_debug.bag
```

回放：

```bash
rosbag play nav_debug.bag
```

使用模拟时间：

```bash
rosparam set use_sim_time true
rosbag play --clock nav_debug.bag
```

## 17.3 C++ 写 bag

```cpp
#include <rosbag/bag.h>
#include <std_msgs/String.h>

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Write);

std_msgs::String msg;
msg.data = "hello";
bag.write("chatter", ros::Time::now(), msg);

bag.close();
```

## 17.4 C++ 读 bag

```cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Read);

for (rosbag::MessageInstance const m : rosbag::View(bag)) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != nullptr) {
        ROS_INFO("%s", s->data.c_str());
    }
}

bag.close();
```

## 17.5 Python 读写 bag

写：

```python
import rosbag
from std_msgs.msg import String

with rosbag.Bag("test.bag", "w") as bag:
    msg = String()
    msg.data = "hello"
    bag.write("chatter", msg)
```

读：

```python
import rosbag

with rosbag.Bag("test.bag", "r") as bag:
    for topic, msg, t in bag.read_messages():
        print(topic, msg, t)
```

---

# 18. rqt 工具箱

常用工具：

```bash
rqt
rqt_graph
rqt_console
rqt_plot
rqt_bag
```

用途：

- `rqt_graph`：查看节点与话题连接关系。
- `rqt_console`：查看日志。
- `rqt_plot`：绘制数值话题曲线。
- `rqt_bag`：图形化查看 bag 数据。

---

# 19. URDF 与机器人模型

## 19.1 作用

URDF 用 XML 描述机器人结构，包括：

- 机器人整体 `robot`
- 连杆 `link`
- 关节 `joint`
- 外观 `visual`
- 碰撞 `collision`
- 惯性 `inertial`

## 19.2 基本 robot

```xml
<robot name="my_robot">
  ...
</robot>
```

xacro：

```xml
<robot name="my_robot" xmlns:xacro="http://wiki.ros.org/xacro">
  ...
</robot>
```

## 19.3 link 示例

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

Gazebo 中不要省略 `collision` 和 `inertial`。

## 19.4 joint 示例

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>
```

常见 joint 类型：

- `fixed`：固定连接
- `continuous`：连续旋转，常用于轮子
- `revolute`：有限角度旋转
- `prismatic`：直线移动

轮子：

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

## 19.5 URDF 验证

```bash
check_urdf my_robot.urdf
urdf_to_graphiz my_robot.urdf
```

---

# 20. xacro

## 20.1 作用

xacro 是 URDF 的宏扩展工具，用于减少重复代码，便于复用尺寸、颜色、惯性、部件等。

## 20.2 property

```xml
<xacro:property name="PI" value="3.1415926"/>
<xacro:property name="base_radius" value="0.2"/>
<xacro:property name="base_length" value="0.16"/>
```

支持表达式：

```xml
<origin xyz="0 0 ${base_length / 2}" rpy="0 0 0"/>
```

## 20.3 macro

```xml
<xacro:macro name="wheel" params="name x y z">
  <link name="${name}">
    ...
  </link>

  <joint name="${name}_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${name}"/>
    <origin xyz="${x} ${y} ${z}" rpy="${PI / 2} 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>

<xacro:wheel name="left_wheel" x="0" y="0.15" z="-0.05"/>
<xacro:wheel name="right_wheel" x="0" y="-0.15" z="-0.05"/>
```

## 20.4 include

```xml
<xacro:include filename="$(find my_robot_description)/urdf/xacro/base.urdf.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/xacro/camera.urdf.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/xacro/laser.urdf.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/xacro/gazebo_plugins.urdf.xacro"/>
```

## 20.5 转换命令

```bash
rosrun xacro xacro my_robot.urdf.xacro > my_robot.urdf
```

launch 中直接加载：

```xml
<param name="robot_description"
       command="$(find xacro)/xacro $(find my_robot_description)/urdf/my_robot.urdf.xacro"/>
```

---

# 21. Rviz 显示机器人模型

基本 launch：

```xml
<launch>
  <param name="robot_description"
         textfile="$(find my_robot_description)/urdf/my_robot.urdf"/>

  <node pkg="joint_state_publisher"
        type="joint_state_publisher"
        name="joint_state_publisher"/>

  <node pkg="robot_state_publisher"
        type="robot_state_publisher"
        name="robot_state_publisher"/>

  <node pkg="joint_state_publisher_gui"
        type="joint_state_publisher_gui"
        name="joint_state_publisher_gui"/>

  <node pkg="rviz"
        type="rviz"
        name="rviz"
        args="-d $(find my_robot_description)/rviz/model.rviz"/>
</launch>
```

核心节点：

- `joint_state_publisher`：发布关节状态。
- `joint_state_publisher_gui`：图形化调整关节。
- `robot_state_publisher`：根据 URDF 和关节状态发布 TF。
- `rviz`：可视化模型。

---

# 22. Gazebo 仿真

## 22.1 基本集成流程

```xml
<launch>
  <param name="robot_description"
         command="$(find xacro)/xacro $(find my_robot_description)/urdf/xacro/my_robot.urdf.xacro"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>

  <node pkg="gazebo_ros"
        type="spawn_model"
        name="spawn_model"
        args="-urdf -model my_robot -param robot_description"/>
</launch>
```

加载 world：

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="world_name" value="$(find my_robot_gazebo)/worlds/my_world.world"/>
</include>
```

## 22.2 Gazebo 材质

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>
```

## 22.3 差速驱动插件

```xml
<gazebo>
  <plugin name="differential_drive_controller"
          filename="libgazebo_ros_diff_drive.so">
    <leftJoint>left_wheel_joint</leftJoint>
    <rightJoint>right_wheel_joint</rightJoint>
    <wheelSeparation>0.3</wheelSeparation>
    <wheelDiameter>0.065</wheelDiameter>
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_footprint</robotBaseFrame>
    <publishTf>true</publishTf>
    <publishWheelTF>true</publishWheelTF>
    <updateRate>30</updateRate>
  </plugin>
</gazebo>
```

测试：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```

## 22.4 激光雷达插件

```xml
<gazebo reference="laser">
  <sensor type="ray" name="laser">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>5.5</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>

    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
      <topicName>scan</topicName>
      <frameName>laser</frameName>
    </plugin>
  </sensor>
</gazebo>
```

Rviz 中添加 `LaserScan`，topic 选 `/scan`。

## 22.5 摄像头与深度相机

常见消息：

```bash
rosmsg info sensor_msgs/Image
rosmsg info sensor_msgs/CompressedImage
rosmsg info sensor_msgs/PointCloud2
```

Rviz 显示：

- `Image`
- `Camera`
- `PointCloud2`

---

# 23. SLAM、地图、定位、导航

## 23.1 导航总体架构

导航需要：

1. 全局地图
2. 机器人定位
3. 全局路径规划
4. 局部路径规划/避障
5. 运动控制
6. 环境感知，通常 `/scan`
7. 正确 TF 树

关键坐标链：

```text
map -> odom -> base_link/base_footprint -> laser/camera
```

## 23.2 gmapping 建图

`slam_gmapping` 使用里程计 + 激光雷达进行二维 SLAM。

输入：

```text
/tf
/scan
```

输出：

```text
/map
/map_metadata
```

launch：

```xml
<launch>
  <node pkg="gmapping"
        type="slam_gmapping"
        name="slam_gmapping"
        output="screen">
    <param name="base_frame" value="base_footprint"/>
    <param name="odom_frame" value="odom"/>
    <param name="map_frame" value="map"/>
  </node>

  <node pkg="rviz"
        type="rviz"
        name="rviz"
        args="-d $(find my_nav)/rviz/gmapping.rviz"/>
</launch>
```

运行顺序：

```bash
roslaunch my_robot_gazebo robot_world.launch
roslaunch my_nav gmapping.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Rviz：

- Fixed Frame：`map`
- Add `Map`，topic `/map`
- Add `RobotModel`
- Add `LaserScan`，topic `/scan`

## 23.3 保存地图 map_server

```bash
rosrun map_server map_saver -f /path/to/nav
```

生成：

```text
nav.pgm
nav.yaml
```

YAML 示例：

```yaml
image: nav.pgm
resolution: 0.050000
origin: [0.000000, 0.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 23.4 加载地图

```xml
<node pkg="map_server"
      type="map_server"
      name="map_server"
      args="$(find my_nav)/map/nav.yaml"/>
```

地图像素概率：

```text
p = (255 - pixel_value) / 255.0
p > occupied_thresh  => occupied
p < free_thresh      => free
其他                 => unknown
```

## 23.5 AMCL 定位

AMCL 在已知地图中估计机器人位姿。

输入：

```text
/scan
/tf
/map
```

输出：

```text
/amcl_pose
/particlecloud
/tf 中的 map -> odom
```

launch：

```xml
<node pkg="amcl" type="amcl" name="amcl" output="screen">
  <param name="odom_model_type" value="diff"/>
  <param name="odom_frame_id" value="odom"/>
  <param name="base_frame_id" value="base_footprint"/>
  <param name="global_frame_id" value="map"/>
  <param name="transform_tolerance" value="1.0"/>
</node>
```

测试 launch：

```xml
<launch>
  <node pkg="map_server"
        type="map_server"
        name="map_server"
        args="$(find my_nav)/map/nav.yaml"/>

  <include file="$(find my_nav)/launch/amcl.launch"/>

  <node pkg="rviz"
        type="rviz"
        name="rviz"
        args="-d $(find my_nav)/rviz/amcl.rviz"/>
</launch>
```

Rviz：

- Add `Map`
- Add `RobotModel`
- Add `PoseArray`，topic `/particlecloud`
- 使用 `2D Pose Estimate` 设置初始位姿

## 23.6 move_base 路径规划

`move_base` 接收目标点并发布速度控制指令。

Action 话题：

```text
/move_base/goal
/move_base/cancel
/move_base/feedback
/move_base/status
/move_base/result
```

简化目标：

```text
/move_base_simple/goal geometry_msgs/PoseStamped
```

输出：

```text
/cmd_vel geometry_msgs/Twist
```

清理代价地图：

```bash
rosservice call /move_base/clear_costmaps
```

launch：

```xml
<launch>
  <node pkg="move_base"
        type="move_base"
        name="move_base"
        respawn="false"
        output="screen"
        clear_params="true">

    <rosparam file="$(find my_nav)/param/costmap_common_params.yaml"
              command="load"
              ns="global_costmap"/>

    <rosparam file="$(find my_nav)/param/costmap_common_params.yaml"
              command="load"
              ns="local_costmap"/>

    <rosparam file="$(find my_nav)/param/local_costmap_params.yaml"
              command="load"/>

    <rosparam file="$(find my_nav)/param/global_costmap_params.yaml"
              command="load"/>

    <rosparam file="$(find my_nav)/param/base_local_planner_params.yaml"
              command="load"/>
  </node>
</launch>
```

## 23.7 costmap_common_params.yaml

```yaml
robot_radius: 0.12
# 或非圆形机器人：
# footprint: [[-0.12, -0.12], [-0.12, 0.12], [0.12, 0.12], [0.12, -0.12]]

obstacle_range: 3.0
raytrace_range: 3.5
inflation_radius: 0.2
cost_scaling_factor: 3.0

observation_sources: scan
scan:
  sensor_frame: laser
  data_type: LaserScan
  topic: scan
  marking: true
  clearing: true
```

## 23.8 global_costmap_params.yaml

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_footprint
  update_frequency: 1.0
  publish_frequency: 1.0
  transform_tolerance: 0.5
  static_map: true
```

## 23.9 local_costmap_params.yaml

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_footprint
  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5
  static_map: false
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
```

## 23.10 base_local_planner_params.yaml

```yaml
TrajectoryPlannerROS:
  max_vel_x: 0.5
  min_vel_x: 0.1
  max_vel_theta: 1.0
  min_in_place_vel_theta: 0.4

  acc_lim_x: 1.0
  acc_lim_theta: 0.6

  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05

  sim_time: 0.8
  vx_samples: 18
  vtheta_samples: 20
```

## 23.11 完整导航 launch

```xml
<launch>
  <node pkg="map_server"
        type="map_server"
        name="map_server"
        args="$(find my_nav)/map/nav.yaml"/>

  <include file="$(find my_nav)/launch/amcl.launch"/>
  <include file="$(find my_nav)/launch/move_base.launch"/>

  <node pkg="rviz"
        type="rviz"
        name="rviz"
        args="-d $(find my_nav)/rviz/nav.rviz"/>
</launch>
```

运行：

```bash
roslaunch my_robot_gazebo robot_world.launch
roslaunch my_nav navigation.launch
```

Rviz 中：

- Fixed Frame：`map`
- 先用 `2D Pose Estimate`
- 再用 `2D Nav Goal`

## 23.12 SLAM + Navigation

```xml
<launch>
  <include file="$(find my_nav)/launch/gmapping.launch"/>
  <include file="$(find my_nav)/launch/move_base.launch"/>
</launch>
```

适合边建图边导航。

## 23.13 导航常见消息

```bash
rosmsg info nav_msgs/MapMetaData
rosmsg info nav_msgs/OccupancyGrid
rosmsg info nav_msgs/Odometry
rosmsg info tf/tfMessage
rosmsg info geometry_msgs/PoseArray
rosmsg info move_base_msgs/MoveBaseActionGoal
rosmsg info nav_msgs/Path
rosmsg info sensor_msgs/LaserScan
rosmsg info sensor_msgs/Image
rosmsg info sensor_msgs/CompressedImage
rosmsg info sensor_msgs/PointCloud2
```

关键消息：

```text
nav_msgs/OccupancyGrid：地图
nav_msgs/Odometry：里程计
geometry_msgs/PoseStamped：目标点
nav_msgs/Path：路径
sensor_msgs/LaserScan：激光雷达
```

## 23.14 深度图像转激光

节点：

```text
depthimage_to_laserscan
```

输入：

```text
image
camera_info
```

输出：

```text
scan
```

launch：

```xml
<node pkg="depthimage_to_laserscan"
      type="depthimage_to_laserscan"
      name="depthimage_to_laserscan"
      output="screen">
  <remap from="image" to="/camera/depth/image_raw"/>
  <remap from="camera_info" to="/camera/depth/camera_info"/>
  <remap from="scan" to="/scan"/>
  <param name="output_frame_id" value="camera_depth_frame"/>
</node>
```

---

# 24. 实体机器人平台设计

## 24.1 总体架构

典型实体机器人分层：

```text
上位机/树莓派：ROS、导航、传感器、决策
下位机/Arduino：电机、编码器、PID、底盘控制
硬件层：电机、电机驱动板、电源、雷达、相机、底盘
```

职责划分：

- Arduino：快速、稳定、低层控制。
- ROS 主机：复杂算法、导航、传感器处理、任务逻辑。

## 24.2 Arduino 基础控制

Arduino 程序结构：

```cpp
void setup() {
  // 初始化
}

void loop() {
  // 循环执行
}
```

电机控制：

```cpp
digitalWrite(DIRA, HIGH);
analogWrite(PWMA, pwm_value);
```

编码器中断：

```cpp
attachInterrupt(digitalPinToInterrupt(pin), encoderISR, RISING);
```

原则：

- ISR 只做计数，不做复杂逻辑。
- 主循环定时计算速度。
- 使用编码器反馈做闭环控制。

## 24.3 PID 电机调速

流程：

1. 读取编码器。
2. 计算实际速度。
3. 与目标速度比较。
4. PID 计算 PWM。
5. 输出到电机。

示例：

```cpp
PID pid(&current_vel, &pwm, &target_vel, kp, ki, kd, DIRECT);
pid.SetMode(AUTOMATIC);
pid.Compute();
analogWrite(PWMA, pwm);
```

调参经验：

- 先调 `Kp`。
- 再加入 `Ki` 消除稳态误差。
- 最后加入 `Kd` 抑制超调。
- `Kp` 太大容易震荡。
- PID 应固定周期运行。

## 24.4 底盘控制链路

典型链路：

```text
/cmd_vel
  -> 上位机控制节点
  -> 串口/rosserial/bridge
  -> Arduino
  -> 左右轮目标速度
  -> PID
  -> PWM
  -> 电机
  -> 编码器反馈
  -> 里程计 /odom
```

写项目时应分离：

- 命令输入
- 电机驱动
- 编码器读取
- PID 控制
- 里程计估计
- 参数调节

## 24.5 传感器集成

常见传感器：

- 激光雷达：`sensor_msgs/LaserScan`
- 摄像头：`sensor_msgs/Image`
- 深度相机：`Image` / `PointCloud2` / 转 `LaserScan`

关键：

- 驱动节点发布正确 topic。
- TF 中有传感器坐标系到 base 的关系。
- Rviz fixed frame 设置正确。
- 导航配置中的 `sensor_frame` 与 TF 一致。

---

# 25. 实体机器人导航

实体导航流程与仿真类似：

1. 启动底盘和传感器。
2. 运行 SLAM 建图。
3. 保存地图。
4. 加载地图。
5. AMCL 定位。
6. move_base 路径规划。

典型命令：

```bash
roslaunch mycar_start start.launch
roslaunch nav gmapping.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

定位：

```bash
roslaunch nav test_amcl.launch
```

核心包：

- `gmapping`
- `map_server`
- `amcl`
- `move_base`

实体机器人调试优先级：

1. `/cmd_vel` 是否能驱动底盘。
2. `/odom` 是否稳定。
3. `/scan` 是否正常。
4. TF 是否完整。
5. 建图是否漂移。
6. AMCL 粒子是否收敛。
7. move_base 是否发布 `/cmd_vel`。

---

# 26. Action 通信

## 26.1 使用场景

Action 适合长任务：

- 导航到目标点
- 机械臂执行动作
- 长时间计算
- 可反馈进度
- 可取消目标

如果任务很快完成，用 Service；如果是持续数据流，用 Topic。

## 26.2 action 文件

位置：

```text
pkg_name/
  action/
    AddInts.action
```

内容：

```action
int32 num
---
int32 result
---
float64 progress_bar
```

三段分别是：

1. goal
2. result
3. feedback

## 26.3 CMake 配置

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  AddInts.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs message_runtime
)
```

## 26.4 C++ 服务端骨架

```cpp
#include <actionlib/server/simple_action_server.h>
#include "demo01_action/AddIntsAction.h"

typedef actionlib::SimpleActionServer<demo01_action::AddIntsAction> Server;

void cb(const demo01_action::AddIntsGoalConstPtr& goal, Server* server) {
    demo01_action::AddIntsFeedback feedback;
    demo01_action::AddIntsResult result;

    // 执行任务，周期性发布 feedback
    server->publishFeedback(feedback);

    // 完成后设置 result
    server->setSucceeded(result);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "addints_server");
    ros::NodeHandle nh;
    Server server(nh, "addInts", boost::bind(&cb, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}
```

## 26.5 C++ 客户端骨架

```cpp
actionlib::SimpleActionClient<demo01_action::AddIntsAction> client(nh, "addInts");
client.waitForServer();
client.sendGoal(goal, done_cb, active_cb, feedback_cb);
```

Action 通信会生成多个 topic：

```text
/goal
/cancel
/status
/feedback
/result
```

---

# 27. dynamic_reconfigure 动态参数

## 27.1 使用场景

运行时调参，例如：

- PID 参数
- 速度限制
- 导航参数
- 阈值
- 滤波参数

## 27.2 cfg 文件

位置：

```text
pkg_name/
  cfg/
    dr.cfg
```

示例：

```python
#!/usr/bin/env python
PACKAGE = "demo02_dr"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("Kp", double_t, 0, "kp", 1.0, 0.0, 10.0)
gen.add("Ki", double_t, 0, "ki", 0.0, 0.0, 10.0)
gen.add("Kd", double_t, 0, "kd", 0.0, 0.0, 10.0)

exit(gen.generate(PACKAGE, "demo", "drConfig"))
```

## 27.3 CMake 配置

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/dr.cfg
)
```

## 27.4 C++ 服务端骨架

```cpp
#include "dynamic_reconfigure/server.h"
#include "demo02_dr/drConfig.h"

void cb(demo02_dr::drConfig &config, uint32_t level) {
    ROS_INFO("Kp=%.2f Ki=%.2f Kd=%.2f", config.Kp, config.Ki, config.Kd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dr_server");

    dynamic_reconfigure::Server<demo02_dr::drConfig> server;
    dynamic_reconfigure::Server<demo02_dr::drConfig>::CallbackType f;
    f = boost::bind(&cb, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}
```

Python 也可使用：

```python
from dynamic_reconfigure.server import Server
from demo02_dr.cfg import drConfig
```

---

# 28. pluginlib

## 28.1 使用场景

pluginlib 用于运行时加载插件，适合可替换算法架构。

适合：

- 路径规划器
- 控制器
- 算法策略
- 行为模块
- 传感器处理模块

核心思想：

1. 定义抽象基类。
2. 实现多个子类插件。
3. 使用宏导出插件。
4. 运行时用 ClassLoader 加载。

## 28.2 导出插件

```cpp
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
```

## 28.3 加载插件

```cpp
#include <pluginlib/class_loader.h>

pluginlib::ClassLoader<polygon_base::RegularPolygon> loader(
  "polygon_pkg", "polygon_base::RegularPolygon");

auto tri = loader.createInstance("polygon_plugins/Triangle");
```

使用场景判断：

- 如果只是固定逻辑，普通类即可。
- 如果需要运行时切换算法，用 pluginlib。

---

# 29. nodelet

## 29.1 使用场景

nodelet 用于同一进程内运行多个组件，减少大消息拷贝和序列化开销。

适合：

- 图像处理
- 点云处理
- 高频传感器链路
- 对性能敏感的数据管线

普通 node 是不同进程，消息跨进程会复制；nodelet 在同一 manager 进程内运行，可以降低开销。

## 29.2 命令

启动 manager：

```bash
rosrun nodelet nodelet manager __name:=mymanager
```

加载 nodelet：

```bash
rosrun nodelet nodelet load nodelet_tutorial_math/Plus mymanager __name:=n1 _value:=100
rosrun nodelet nodelet load nodelet_tutorial_math/Plus mymanager __name:=n2 _value:=-50 /n2/in:=/n1/out
```

## 29.3 类模板

```cpp
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace nodelet_demo_ns {

class MyPlus : public nodelet::Nodelet {
  void onInit() override {
    // 初始化发布者、订阅者、参数
  }
};

}

PLUGINLIB_EXPORT_CLASS(nodelet_demo_ns::MyPlus, nodelet::Nodelet)
```

CMake：

```cmake
add_library(mynodeletlib src/my_plus.cpp)
target_link_libraries(mynodeletlib ${catkin_LIBRARIES})
```

---

# 30. ROS 项目开发检查清单

## 30.1 新建通信包

1. 创建包：

```bash
catkin_create_pkg pkg_name roscpp rospy std_msgs message_generation message_runtime
```

2. 如果有自定义消息：创建 `msg/*.msg`。
3. 如果有自定义服务：创建 `srv/*.srv`。
4. 如果有 Action：创建 `action/*.action`。
5. 修改 `package.xml`。
6. 修改 `CMakeLists.txt`。
7. 写 C++ 到 `src/`，Python 到 `scripts/`。
8. 写 launch 到 `launch/`。
9. 编译：

```bash
catkin_make
source devel/setup.bash
```

10. 调试：

```bash
rosnode list
rostopic list
rosservice list
rosparam list
rqt_graph
```

## 30.2 仿真机器人检查

1. URDF/xacro 是否正确：

```bash
check_urdf robot.urdf
```

2. Rviz 是否能显示 RobotModel。
3. TF 是否完整：

```bash
rosrun tf2_tools view_frames.py
```

4. Gazebo 是否加载模型。
5. `/cmd_vel` 是否能控制运动。
6. `/odom` 是否正常。
7. `/scan` 是否正常。
8. Rviz fixed frame 是否正确。

## 30.3 导航检查

1. TF 链必须存在：

```text
map -> odom -> base_footprint/base_link -> laser
```

2. 雷达：

```bash
rostopic echo /scan
```

3. 里程计：

```bash
rostopic echo /odom
```

4. 建图：

```bash
roslaunch my_nav gmapping.launch
```

5. 保存地图：

```bash
rosrun map_server map_saver -f maps/nav
```

6. AMCL：检查 `/particlecloud`。
7. move_base：检查 `/cmd_vel`。
8. 清理代价地图：

```bash
rosservice call /move_base/clear_costmaps
```

9. 记录调试 bag：

```bash
rosbag record /tf /tf_static /odom /scan /map /cmd_vel /move_base_simple/goal -O nav_debug.bag
```

## 30.4 实体机器人检查

1. 电源、电机驱动、编码器连线正确。
2. Arduino 能单独控制电机。
3. 编码器计数方向正确。
4. PID 能稳定跟踪目标速度。
5. 上位机能发送 `/cmd_vel`。
6. 底盘能发布 `/odom`。
7. 雷达能发布 `/scan`。
8. 静态 TF 配好传感器位置。
9. 建图时地图不严重漂移。
10. AMCL 能收敛。
11. move_base 能发速度指令。

---

# 31. 未来写 ROS 项目时的默认方案

如果用户没有特殊要求，写 ROS 项目时按以下默认：

- 使用 ROS 1 catkin 包。
- C++ 节点放 `src/`。
- Python 节点放 `scripts/`，加 shebang 与执行权限。
- launch 文件放 `launch/`。
- 参数 YAML 放 `config/`、`cfg/` 或 `param/`。
- 自定义消息放 `msg/`。
- 自定义服务放 `srv/`。
- 自定义 action 放 `action/`。
- 机器人模型用 `urdf/xacro/`。
- Rviz 配置放 `rviz/`。
- 地图放 `maps/`。
- Gazebo world 放 `worlds/`。

默认设计判断：

```text
传感器数据        -> Topic
速度控制          -> Topic /cmd_vel
里程计            -> Topic /odom + TF odom->base
一次性命令        -> Service
长任务            -> Action
可调参数          -> dynamic_reconfigure
固定配置          -> YAML + rosparam
坐标关系          -> TF2
模型              -> URDF/xacro
仿真              -> Gazebo
可视化            -> Rviz/rqt
导航              -> gmapping/map_server/amcl/move_base
```

---

# 32. 最重要的排错原则

ROS 项目出问题时，按以下顺序查：

1. 环境是否 source 正确：

```bash
source devel/setup.bash
rospack find 包名
```

2. 节点是否启动：

```bash
rosnode list
rosnode info /node
```

3. 话题是否存在：

```bash
rostopic list
rostopic info /topic
rostopic echo /topic
```

4. 消息类型是否匹配：

```bash
rostopic type /topic
rosmsg show 类型
```

5. 服务是否存在：

```bash
rosservice list
rosservice info /service
```

6. 参数是否加载：

```bash
rosparam list
rosparam get /param
```

7. TF 是否连通：

```bash
rosrun tf2_tools view_frames.py
rosrun tf tf_echo frame1 frame2
```

8. launch 是否命名空间或重映射错误。
9. 多工作空间是否包名冲突。
10. 导航中先看 TF、`/odom`、`/scan`，再看算法参数。

---

# 33. 一句话总纲

ROS 项目开发的核心不是把所有功能写进一个程序，而是把机器人系统拆成多个节点，用 Topic、Service、Parameter、Action、TF、launch、URDF/xacro、Gazebo、Rviz、导航栈等标准机制组合起来；调试时先确认包路径、节点、话题、服务、参数、TF，再处理业务逻辑。
