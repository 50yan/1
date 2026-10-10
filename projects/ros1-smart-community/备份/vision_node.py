#!/usr/bin/env python3
import os
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

script_dir = os.path.dirname(os.path.abspath(globals().get('__file__', globals().get('python_script', sys.argv[0]))))
sys.path.insert(0, script_dir)
from vision_tools import analyze_image


class VisionNode:
    def __init__(self):
        self.label = rospy.get_param("~label", "people")
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/.ros/zhihui_shequ/vision_node"))
        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("~result", String, queue_size=10)
        self.sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/image_raw"), Image, self.on_image, queue_size=1)

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("image conversion failed: %s", exc)
            return
        result = analyze_image(self.label, image)
        self.pub.publish(result["summary"])
        if rospy.get_param("~save_frames", False):
            path = os.path.join(self.output_dir, "%s_%s.jpg" % (time.strftime("%Y%m%d_%H%M%S"), self.label))
            cv2.imwrite(path, result["annotated"])


def main():
    rospy.init_node("vision_node")
    VisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()

