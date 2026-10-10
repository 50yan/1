#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from community_interfaces.msg import PerceptionResult


def main():
    rospy.init_node("mannequin_detector_node")
    bridge = CvBridge()
    publisher = rospy.Publisher("/perception/result", PerceptionResult, queue_size=10)
    def callback(message):
        image = bridge.imgmsg_to_cv2(message, "bgr8")
        mask = cv2.inRange(image, (50, 0, 0), (255, 160, 120))
        count = sum(1 for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0] if cv2.contourArea(contour) >= 250)
        result = PerceptionResult(header=message.header, target_type="mannequin", class_name="mannequin", confidence=0.7 if count else 0.0, count=count)
        publisher.publish(result)
    rospy.Subscriber("/camera/image_raw", Image, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
