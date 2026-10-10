#!/usr/bin/env python3
import time
import actionlib
import rospy

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


PATROL_POINTS = [
    {
        "name": "point_1",
        "x": -0.8889723420,
        "y": 1.6374720335,
        "qz": -0.7090742879,
        "qw": 0.7051337846,
    },
    {
        "name": "point_2",
        "x": -1.6633207798,
        "y": 1.0,
        "qz": -0.0168198201,
        "qw": 0.9998585368,
    },
    {
        "name": "point_3",
        "x": -0.8467646241,
        "y": 0.61553966999,
        "qz": 0.7095989107,
        "qw": 0.7046058373,
    },
    {
        "name": "point_4",
        "x": -0.8390713334083557,
        "y": 0.526359498500824,
        "qz": -0.7024285819568559,
        "qw": 0.711754232337315,
    },
    {
        "name": "point_5",
        "x": -0.1176491230726242,
        "y": -0.08616995811462402,
        "qz": 0.9999999238534443,
        "qw": 0.0003902474926939215,
    },
    {
        "name": "point_6",
        "x": 1.0873372554779053,
        "y": -1.6492003202438354,
        "qz": -0.005484887371207373,
        "qw": 0.99998495789213,
    },
    {
        "name": "point_7",
        "x": 1.058310627937317,
        "y": -1.1196959018707275,
        "qz": 0.02376246286492284,
        "qw": 0.9997176328136826,
    },
    {
        "name": "point_8",
        "x": 1.0487483739852905,
        "y": -0.48562490940093994,
        "qz": 0.00047872268978167845,
        "qw": 0.9999998854122866,
    },
    {
        "name": "point_9",
        "x": 1.755853533744812,
        "y": 1.7394684553146362,
        "qz": 0.9999878359378007,
        "qw": 0.0049323398538695575,
    },
]


def make_goal(point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point["x"]
    goal.target_pose.pose.position.y = point["y"]
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = point["qz"]
    goal.target_pose.pose.orientation.w = point["qw"]
    return goal


def main():
    rospy.init_node("ceshi3_patrol")
    
    deadline = time.monotonic() + 10
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
      if time.monotonic() >= deadline:
          rospy.logerr("未收到 /clock，请检查 Gazebo 是否运行")
          return
      time.sleep(0.1)
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("move_base is not available")
        return

    for index, point in enumerate(PATROL_POINTS, start=1):
        if rospy.is_shutdown():
            return

        rospy.loginfo(
            "Sending %s (%d/%d): x=%.3f, y=%.3f",
            point["name"],
            index,
            len(PATROL_POINTS),
            point["x"],
            point["y"],
        )
        client.send_goal(make_goal(point))

        if not client.wait_for_result(rospy.Duration(180.0)):
            rospy.logerr("Timed out at %s; cancelling goal", point["name"])
            client.cancel_goal()
            return

        state = client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr("Failed at %s; action state=%d", point["name"], state)
            return

        rospy.loginfo("Reached %s", point["name"])
        rospy.sleep(2.0)

    rospy.loginfo("Three-point patrol completed")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
