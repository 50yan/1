#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    rospy.init_node("visual_overlay_node")
    bridge = CvBridge()
    publisher = rospy.Publisher("/perception/annotated_image", Image, queue_size=1)
    def callback(message):
        image = bridge.imgmsg_to_cv2(message, "bgr8")
        publisher.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
    rospy.Subscriber("/camera/image_raw", Image, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
