#!/usr/bin/env python3
import rospy
from community_interfaces.msg import MissionStatus, PerceptionResult


def main():
    rospy.init_node("result_reporter")
    rospy.Subscriber("/mission/status", MissionStatus, lambda m: rospy.loginfo("MISSION %s %s %s", m.state, m.waypoint, m.detail))
    rospy.Subscriber("/perception/result", PerceptionResult, lambda m: rospy.loginfo("PERCEPTION %s %s count=%d ocr=%s", m.target_type, m.class_name, m.count, m.ocr_text))
    rospy.spin()


if __name__ == "__main__":
    main()
