#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Quaternion


def quaternion_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def main():
    rospy.init_node("reset_spawn_model")
    model_name = rospy.get_param("~model_name", "smart_car")
    robot_description_param = rospy.get_param("~robot_description_param", "/robot_description")
    reference_frame = rospy.get_param("~reference_frame", "world")
    robot_namespace = rospy.get_param("~robot_namespace", "")
    x = float(rospy.get_param("~x", 4.15))
    y = float(rospy.get_param("~y", 3.65))
    z = float(rospy.get_param("~z", 0.22))
    yaw = float(rospy.get_param("~yaw", 3.14159))

    if not rospy.has_param(robot_description_param):
        raise RuntimeError("Missing robot description parameter: %s" % robot_description_param)
    model_xml = rospy.get_param(robot_description_param)

    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    try:
        result = delete_model(model_name)
        if result.success:
            rospy.loginfo("Deleted existing Gazebo model: %s", model_name)
        else:
            rospy.loginfo("No existing Gazebo model deleted for %s: %s", model_name, result.status_message)
    except Exception as exc:
        rospy.logwarn("Delete existing model failed for %s: %s", model_name, exc)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = quaternion_from_yaw(yaw)

    result = spawn_model(model_name, model_xml, robot_namespace, pose, reference_frame)
    if not result.success:
        raise RuntimeError("Spawn model failed: %s" % result.status_message)
    rospy.loginfo("Spawned Gazebo model %s at x=%.2f y=%.2f z=%.2f yaw=%.2f", model_name, x, y, z, yaw)


if __name__ == "__main__":
    main()