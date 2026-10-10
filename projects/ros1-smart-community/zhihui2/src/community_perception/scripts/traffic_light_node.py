#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from community_interfaces.msg import PerceptionResult


def main():
    rospy.init_node("traffic_light_node")
    state_pub = rospy.Publisher("/perception/traffic_light_state", String, queue_size=1, latch=True)
    result_pub = rospy.Publisher("/perception/result", PerceptionResult, queue_size=10)
    def callback(message):
        state = message.data.lower()
        state_pub.publish(state)
        result = PerceptionResult()
        result.header.stamp = rospy.Time.now()
        result.target_type = "traffic_light"
        result.class_name = state
        result.confidence = 1.0
        result.count = 1
        result_pub.publish(result)
    rospy.Subscriber("/gazebo/traffic_light_state", String, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
