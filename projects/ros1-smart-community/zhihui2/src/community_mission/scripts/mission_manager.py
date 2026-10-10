#!/usr/bin/env python3
import math
import os
import rospy
import yaml
import actionlib
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from community_interfaces.msg import MissionStatus
from community_interfaces.srv import StartMission, StartMissionResponse, StopMission, StopMissionResponse


class MissionManager:
    def __init__(self):
        self.active = False
        self.traffic_state = "unknown"
        self.status = rospy.Publisher("/mission/status", MissionStatus, queue_size=10, latch=True)
        self.stop_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/perception/traffic_light_state", String, self.on_light, queue_size=1)
        rospy.Service("/mission/start", StartMission, self.start)
        rospy.Service("/mission/stop", StopMission, self.stop)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def on_light(self, message):
        self.traffic_state = message.data.lower()

    def publish(self, state, waypoint="", detail=""):
        self.status.publish(MissionStatus(state=state, waypoint=waypoint, detail=detail, active=self.active))

    def start(self, request):
        if not self.active:
            self.active = True
            rospy.Timer(rospy.Duration(0.1), self.run, oneshot=True)
        return StartMissionResponse(True, "mission accepted")

    def stop(self, _request):
        self.active = False
        self.client.cancel_all_goals()
        self.stop_pub.publish(Twist())
        self.publish("FINISHED", detail="stopped by service")
        return StopMissionResponse(True, "mission stopped")

    def goal(self, point):
        result = MoveBaseGoal()
        result.target_pose.header.frame_id = "map"
        result.target_pose.header.stamp = rospy.Time.now()
        result.target_pose.pose.position.x = point["x"]
        result.target_pose.pose.position.y = point["y"]
        result.target_pose.pose.orientation = Quaternion(0, 0, math.sin(point["yaw"] / 2.0), math.cos(point["yaw"] / 2.0))
        return result

    def run(self, _event):
        route = os.path.join(rospy.get_param("~route_package", ""), "patrol_route.yaml")
        if not route or not os.path.isfile(route):
            import rospkg
            route = os.path.join(rospkg.RosPack().get_path("community_navigation"), "waypoints", "patrol_route.yaml")
        with open(route, "r", encoding="utf-8") as stream:
            points = yaml.safe_load(stream)["waypoints"]
        self.client.wait_for_server()
        self.publish("INIT")
        for point in points:
            if not self.active:
                return
            self.publish("NAVIGATE_TO_POINT", point["name"])
            self.client.send_goal(self.goal(point))
            self.publish("WAIT_FOR_ARRIVAL", point["name"])
            self.client.wait_for_result(rospy.Duration(rospy.get_param("~arrival_timeout", 90.0)))
            self.publish("PERCEPTION_TASK", point["name"], point["task"])
            if point["task"] == "traffic_light":
                self.publish("DECISION", point["name"], "waiting for green")
                while self.active and self.traffic_state != "green" and not rospy.is_shutdown():
                    self.stop_pub.publish(Twist())
                    rospy.sleep(0.2)
            self.publish("NEXT_WAYPOINT", point["name"])
        self.active = False
        self.publish("PARK", "park")
        self.publish("FINISHED")


if __name__ == "__main__":
    rospy.init_node("mission_manager")
    MissionManager()
    rospy.spin()
