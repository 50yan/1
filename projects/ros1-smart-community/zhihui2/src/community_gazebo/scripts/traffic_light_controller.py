#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("traffic_light_controller")
    state = rospy.get_param("~initial_state", "green").lower()
    period = float(rospy.get_param("~period", 15.0))
    publisher = rospy.Publisher("/gazebo/traffic_light_state", String, queue_size=1, latch=True)
    rate = rospy.Rate(2)
    next_switch = rospy.Time.now() + rospy.Duration(period)
    while not rospy.is_shutdown():
        publisher.publish(state)
        if period > 0 and rospy.Time.now() >= next_switch:
            state = "red" if state == "green" else "green"
            next_switch = rospy.Time.now() + rospy.Duration(period)
        rate.sleep()


if __name__ == "__main__":
    main()
