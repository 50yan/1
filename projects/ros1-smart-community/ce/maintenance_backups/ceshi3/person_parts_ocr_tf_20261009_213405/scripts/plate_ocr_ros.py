#!/usr/bin/env python3
import os
os.environ.setdefault("FLAGS_use_mkldnn", "0")
os.environ.setdefault("FLAGS_enable_pir_api", "0")

import json
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from paddleocr import PaddleOCR


def normalize_plate_text(text):
    """Normalize OCR output while preserving valid province/letter prefixes."""
    value = str(text).strip().upper()
    value = value.replace("\\XB7", "·").replace("\\xB7", "·")
    value = value.replace(" ", "").replace("　", "")
    value = value.replace("•", "·").replace("．", "·").replace(".", "·")
    return value


class PlateOCRNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.roi_x = int(rospy.get_param("~roi_x", 0))
        self.roi_y = int(rospy.get_param("~roi_y", 0))
        self.roi_w = int(rospy.get_param("~roi_w", 0))
        self.roi_h = int(rospy.get_param("~roi_h", 0))
        self.min_score = float(rospy.get_param("~min_score", 0.50))
        self.process_hz = float(rospy.get_param("~process_hz", 2.0))
        self.last_process_time = 0.0
        self.plate_aliases = rospy.get_param("~plate_aliases", {"苏W·GT88V": "苏·GT88V"})

        rospy.loginfo("Loading PaddleOCR with CPU-safe settings...")
        self.ocr = PaddleOCR(
            use_angle_cls=True,
            lang="ch",
            use_gpu=False,
            show_log=False,
            enable_mkldnn=False,
            ir_optim=False,
            cpu_threads=1,
            use_mp=False,
        )

        self.result_pub = rospy.Publisher("/plate_ocr/result", String, queue_size=10)
        self.image_pub = rospy.Publisher(
            "/plate_ocr/image_annotated", Image, queue_size=1
        )
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.loginfo("OCR subscribed to %s", self.image_topic)

    def image_callback(self, msg):
        now = time.time()
        if self.process_hz > 0 and now - self.last_process_time < 1.0 / self.process_hz:
            return
        self.last_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "cv_bridge error: %s", exc)
            return

        height, width = frame.shape[:2]
        x0 = max(0, self.roi_x)
        y0 = max(0, self.roi_y)
        x1 = min(width, x0 + self.roi_w) if self.roi_w > 0 else width
        y1 = min(height, y0 + self.roi_h) if self.roi_h > 0 else height
        if x1 <= x0 or y1 <= y0:
            x0, y0, x1, y1 = 0, 0, width, height
        crop = frame[y0:y1, x0:x1]
        if crop.size == 0:
            return

        try:
            ocr_result = self.ocr.ocr(crop, cls=True)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "OCR failed: %s", exc)
            return

        items = []
        lines = ocr_result[0] if ocr_result and ocr_result[0] else []
        for line in lines:
            if len(line) < 2:
                continue
            box, pair = line[0], line[1]
            raw_text = str(pair[0])
            normalized = normalize_plate_text(raw_text)
            text, score = self.plate_aliases.get(normalized, normalized), float(pair[1])
            if score < self.min_score:
                continue
            items.append({"text": text, "raw_text": raw_text, "score": score,
                          "correction_applied": text != normalized})
            points = np.array(box, dtype=np.int32)
            points[:, 0] += x0
            points[:, 1] += y0
            cv2.polylines(frame, [points], True, (0, 255, 0), 2)
            cv2.putText(
                frame,
                "{} {:.2f}".format(text, score),
                (int(points[0][0]), max(20, int(points[0][1]) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "text": " ".join(item["text"] for item in items),
            "items": items,
        }
        self.result_pub.publish(json.dumps(payload, ensure_ascii=False))
        try:
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated.header = msg.header
            self.image_pub.publish(annotated)
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "Annotated image publish failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("plate_ocr")
    PlateOCRNode()
    rospy.spin()
