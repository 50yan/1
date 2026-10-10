#!/usr/bin/env python3
import math
import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


def quaternion(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    rospy.init_node("waypoint_navigator")
    route = rospy.get_param("~route", "$(find community_navigation)/waypoints/patrol_route.yaml")
    if route.startswith("$(find"):
        import rospkg
        route = rospkg.RosPack().get_path("community_navigation") + "/waypoints/patrol_route.yaml"
    with open(route, "r", encoding="utf-8") as stream:
        points = yaml.safe_load(stream)["waypoints"]
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    for point in points:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point["x"]
        goal.target_pose.pose.position.y = point["y"]
        goal.target_pose.pose.orientation = quaternion(point["yaw"])
        rospy.loginfo("Navigating to %s", point["name"])
        client.send_goal(goal)
        client.wait_for_result()


if __name__ == "__main__":
    main()
