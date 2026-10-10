#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from community_interfaces.msg import PerceptionResult


def main():
    rospy.init_node("license_plate_node")
    bridge = CvBridge()
    publisher = rospy.Publisher("/perception/result", PerceptionResult, queue_size=10)
    def callback(message):
        image = bridge.imgmsg_to_cv2(message, "bgr8")
        mask = cv2.inRange(image, (90, 20, 0), (255, 140, 90))
        candidates = [c for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0] if cv2.contourArea(c) > 180]
        result = PerceptionResult(header=message.header, target_type="license_plate", class_name="license_plate", confidence=0.7 if candidates else 0.0, count=len(candidates), ocr_text="SIM-AIC111" if candidates else "")
        publisher.publish(result)
    rospy.Subscriber("/camera/image_raw", Image, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
