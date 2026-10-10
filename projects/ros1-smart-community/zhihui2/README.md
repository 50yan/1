# Smart Community ROS Workspace

ROS1 Catkin workspace for a Gazebo smart-community patrol robot. The project is organised as one functional ROS package per concern, so launch files resolve resources using `$(find package_name)` rather than workspace-relative paths.

## Requirements

- Ubuntu 20.04 and ROS Noetic
- Gazebo 11, `ros-noetic-gazebo-ros`, `gmapping`, `amcl`, `move-base`, `map-server`, `rviz`
- Python 3 packages: `python3-opencv`, `python3-yaml`, `python3-rospkg`

The current lightweight perception implementation uses OpenCV colour segmentation and the Gazebo traffic-light state. For a competition model, place a compact `mannequin_yolov8.onnx` in `src/community_perception/models/`, retain its source/checksum or a download script, then replace the detector implementation without changing the public topics or `community_interfaces` message contract.

## Packages

- `community_description`: differential-drive robot, laser, camera, IMU and complete `base_footprint -> base_link -> {laser_link,camera_link,imu_link}` TF subtree.
- `community_gazebo`: world, materials and Gazebo models. The scene contains lane lines, collision-enabled traffic light, mannequin, licence-plate car, building and road sign models.
- `community_slam`: gmapping, saved map and AMCL localisation.
- `community_navigation`: move_base configuration and named patrol waypoints.
- `community_perception`: raw camera, annotated image and structured perception outputs.
- `community_mission`: mission state machine and terminal reporter.
- `community_interfaces`: stable messages/services shared between perception and the mission layer.

## Build

```bash
cd /home/ros1/zhihui2
./scripts/build.sh
source devel/setup.bash
```

## Start Modes

Simulation only:

```bash
roslaunch community_gazebo simulation.launch
```

Mapping:

```bash
roslaunch community_slam mapping.launch
```

Saved-map localisation and navigation:

```bash
roslaunch community_gazebo simulation.launch
roslaunch community_slam localization.launch
roslaunch community_navigation navigation.launch
```

Complete patrol demo:

```bash
./scripts/run_demo.sh
rosservice call /mission/start "route_name: 'patrol_route'"
```

The expected result is Gazebo simulation with robot sensors, a stable `map -> odom -> base_footprint` chain, RViz navigation, `/perception/annotated_image`, structured `/perception/result` messages, and mission states `INIT -> NAVIGATE_TO_POINT -> WAIT_FOR_ARRIVAL -> PERCEPTION_TASK -> DECISION -> NEXT_WAYPOINT -> PARK -> FINISHED` on `/mission/status`. At the traffic-light waypoint the state machine sends zero velocity while the reported signal remains red, then resumes after green.

For recording, start components in this order: Gazebo and robot, mapping or localisation, navigation, perception, then mission. Use `./scripts/record_demo.sh output_name` after the topic check passes.
