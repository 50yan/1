cmake_minimum_required(VERSION 3.0.2)
project(zhihui_shequ_sim)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  rosgraph_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  xacro
  robot_state_publisher
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_controller.py
  scripts/reset_spawn_model.py
  scripts/navigation_status_window.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  maps
  media
  rviz
  urdf
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
