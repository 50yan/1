#!/usr/bin/env python3
"""ROS1 camera node: crop a traffic-light box and classify its active color.

This first version uses a fixed pixel ROI. After the YOLO traffic-light detector
is connected, replace the fixed ROI with the YOLO bounding box.
"""

import json
import os
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TrafficLightHSV:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/vision/traffic_light/debug"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/vision/traffic_light/state"
        )

        # Pixel coordinates in the original camera image. Set these after
        # viewing the image in rqt_image_view.
        self.x = int(rospy.get_param("~x", 0))
        self.y = int(rospy.get_param("~y", 0))
        self.w = int(rospy.get_param("~w", 0))
        self.h = int(rospy.get_param("~h", 0))

        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.01))
        self.min_value = int(rospy.get_param("~min_value", 100))
        self.min_saturation = int(rospy.get_param("~min_saturation", 80))
        self.vote_size = max(1, int(rospy.get_param("~vote_size", 5)))
        self.save_debug = bool(rospy.get_param("~save_debug", True))
        self.save_dir = os.path.expanduser(
            rospy.get_param(
                "~save_dir", "/home/ros1/ce/src/vision_data/results/traffic_hsv"
            )
        )
        if self.save_debug:
            os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.votes = []
        self.frame = 0
        self.last_log = 0.0
        rospy.loginfo("HSV node subscribed to %s", self.image_topic)
        rospy.loginfo("ROI x=%d y=%d w=%d h=%d", self.x, self.y, self.w, self.h)

    @staticmethod
    def color_masks(hsv, min_saturation, min_value):
        red1 = cv2.inRange(
            hsv, np.array([0, min_saturation, min_value]), np.array([10, 255, 255])
        )
        red2 = cv2.inRange(
            hsv, np.array([170, min_saturation, min_value]), np.array([179, 255, 255])
        )
        masks = {
            "red": cv2.bitwise_or(red1, red2),
            "yellow": cv2.inRange(
                hsv,
                np.array([15, min_saturation, min_value]),
                np.array([40, 255, 255]),
            ),
            "green": cv2.inRange(
                hsv,
                np.array([40, min_saturation, min_value]),
                np.array([95, 255, 255]),
            ),
        }
        return masks

    def classify(self, roi):
        # OpenCV reads images as BGR. HSV conversion is the key operation.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        masks = self.color_masks(hsv, self.min_saturation, self.min_value)

        # Traffic lights are assumed to be vertically arranged: red, yellow,
        # green. Check each third separately to avoid counting the whole box.
        height = roi.shape[0]
        slots = {
            "red": roi[: max(1, height // 3)],
            "yellow": roi[height // 3 : max(height // 3 + 1, 2 * height // 3)],
            "green": roi[2 * height // 3 :],
        }
        scores = {}
        for color, slot in slots.items():
            slot_hsv = cv2.cvtColor(slot, cv2.COLOR_BGR2HSV)
            mask = self.color_masks(
                slot_hsv, self.min_saturation, self.min_value
            )[color]
            ratio = float(np.count_nonzero(mask)) / float(mask.size)
            scores[color] = ratio

        best_color = max(scores, key=scores.get)
        if scores[best_color] < self.min_area_ratio:
            return "unknown", scores, hsv
        # Require the winner to be sufficiently stronger than the runner-up.
        ordered = sorted(scores.values(), reverse=True)
        if len(ordered) > 1 and ordered[0] < ordered[1] * 1.20:
            return "unknown", scores, hsv
        return best_color, scores, hsv

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "cv_bridge error: %s", error)
            return

        height, width = image.shape[:2]
        x, y, w, h = self.x, self.y, self.w, self.h
        if w <= 0 or h <= 0:
            rospy.logwarn_throttle(
                5.0,
                "ROI is not configured. Set ~x ~y ~w ~h from the camera image; "
                "received %dx%d",
                width,
                height,
            )
            return
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(width, x + w), min(height, y + h)
        if x2 <= x1 or y2 <= y1:
            rospy.logwarn_throttle(5.0, "ROI is outside the image")
            return

        roi = image[y1:y2, x1:x2]
        state, scores, _hsv = self.classify(roi)
        self.votes.append(state)
        self.votes = self.votes[-self.vote_size :]
        confirmed = state
        if self.votes:
            confirmed = max(set(self.votes), key=self.votes.count)

        annotated = image.copy()
        color_bgr = {
            "red": (0, 0, 255),
            "yellow": (0, 220, 255),
            "green": (0, 220, 0),
            "unknown": (160, 160, 160),
        }[confirmed]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color_bgr, 2)
        label = "{} R{:.3f} Y{:.3f} G{:.3f}".format(
            confirmed, scores["red"], scores["yellow"], scores["green"]
        )
        cv2.putText(
            annotated,
            label,
            (x1, max(25, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color_bgr,
            2,
            cv2.LINE_AA,
        )

        self.frame += 1
        result = {
            "frame": self.frame,
            "state_raw": state,
            "state": confirmed,
            "scores": {key: round(value, 5) for key, value in scores.items()},
            "roi": [x1, y1, x2, y2],
            "stamp": message.header.stamp.to_sec(),
        }
        self.state_pub.publish(String(data=json.dumps(result, ensure_ascii=True)))
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "debug image publish error: %s", error)

        if self.save_debug and self.frame % 10 == 0:
            path = os.path.join(self.save_dir, "frame_{:06d}.jpg".format(self.frame))
            cv2.imwrite(path, annotated)

        now = time.monotonic()
        if now - self.last_log > 2.0:
            rospy.loginfo("traffic_light=%s scores=%s", confirmed, result["scores"])
            self.last_log = now


if __name__ == "__main__":
    rospy.init_node("traffic_light_hsv_node")
    TrafficLightHSV()
    rospy.spin()
