#!/usr/bin/env python3
"""ROS1 smoke test for the community/noncommunity ONNX detector."""

import json
import os
import time

import cv2
import numpy as np
import onnxruntime as ort
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


CLASS_NAMES = ("community", "noncommunity")
INPUT_SIZE = 640


def letterbox(image):
    height, width = image.shape[:2]
    ratio = min(INPUT_SIZE / float(width), INPUT_SIZE / float(height))
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    pad_x = (INPUT_SIZE - resized_width) / 2.0
    pad_y = (INPUT_SIZE - resized_height) / 2.0
    canvas = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.uint8)
    left = int(round(pad_x - 0.1))
    top = int(round(pad_y - 0.1))
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return canvas, ratio, float(left), float(top)


def iou(box, other):
    x1 = max(box[0], other[0])
    y1 = max(box[1], other[1])
    x2 = min(box[0] + box[2], other[0] + other[2])
    y2 = min(box[1] + box[3], other[1] + other[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = box[2] * box[3] + other[2] * other[3] - intersection
    return intersection / union if union > 0 else 0.0


def nms(boxes, scores, threshold):
    order = sorted(range(len(scores)), key=lambda index: scores[index], reverse=True)
    kept = []
    while order:
        current = order.pop(0)
        kept.append(current)
        order = [index for index in order if iou(boxes[current], boxes[index]) < threshold]
    return kept


class YoloRosTest:
    def __init__(self):
        model_path = rospy.get_param(
            "~model",
            "/home/ros1/ce/src/vision_data/vision_models/person_type_xz4_ros.onnx",
        )
        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.confidence_threshold = float(rospy.get_param("~confidence", 0.35))
        self.iou_threshold = float(rospy.get_param("~iou", 0.45))
        self.save_images = bool(rospy.get_param("~save_images", True))
        self.save_interval = max(1, int(rospy.get_param("~save_interval", 10)))
        self.save_dir = rospy.get_param(
            "~save_dir", "/home/ros1/ce/src/vision_data/results/yolo_test"
        )
        if not os.path.isfile(model_path):
            raise RuntimeError("Model does not exist: {}".format(model_path))
        if self.save_images:
            os.makedirs(self.save_dir, exist_ok=True)

        self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        output = self.session.get_outputs()[0]
        rospy.loginfo("YOLO ONNX loaded: %s", model_path)
        rospy.loginfo("input=%s output=%s", self.input_name, output.name)

        self.bridge = CvBridge()
        self.frame_number = 0
        self.image_publisher = rospy.Publisher(
            "/vision/detections/image", Image, queue_size=1
        )
        self.result_publisher = rospy.Publisher(
            "/vision/detections/result", String, queue_size=10
        )
        self.subscription = rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )
        rospy.loginfo("Subscribed to %s", image_topic)

    def detect(self, image):
        padded, ratio, pad_x, pad_y = letterbox(image)
        blob = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        output = self.session.run(None, {self.input_name: blob[None, ...]})[0]
        prediction = output[0].T

        boxes = []
        scores = []
        class_ids = []
        image_height, image_width = image.shape[:2]
        for row in prediction:
            class_id = int(np.argmax(row[4:]))
            confidence = float(row[4 + class_id])
            if confidence < self.confidence_threshold:
                continue
            center_x, center_y, box_width, box_height = row[:4]
            x1 = (float(center_x) - float(box_width) / 2.0 - pad_x) / ratio
            y1 = (float(center_y) - float(box_height) / 2.0 - pad_y) / ratio
            x2 = (float(center_x) + float(box_width) / 2.0 - pad_x) / ratio
            y2 = (float(center_y) + float(box_height) / 2.0 - pad_y) / ratio
            x1 = max(0.0, min(float(image_width - 1), x1))
            y1 = max(0.0, min(float(image_height - 1), y1))
            x2 = max(x1 + 1.0, min(float(image_width), x2))
            y2 = max(y1 + 1.0, min(float(image_height), y2))
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(confidence)
            class_ids.append(class_id)

        keep = nms(boxes, scores, self.iou_threshold)
        return [
            {
                "class_id": class_ids[index],
                "class": CLASS_NAMES[class_ids[index]],
                "confidence": round(scores[index], 4),
                "bbox": [int(round(value)) for value in boxes[index]],
            }
            for index in keep
        ]

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "cv_bridge: %s", error)
            return

        started = time.time()
        detections = self.detect(image)
        annotated = image.copy()
        for detection in detections:
            x, y, width, height = detection["bbox"]
            color = (0, 220, 0) if detection["class_id"] == 0 else (0, 0, 255)
            cv2.rectangle(annotated, (x, y), (x + width, y + height), color, 2)
            text = "{} {:.2f}".format(detection["class"], detection["confidence"])
            cv2.putText(annotated, text, (x, max(20, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2, cv2.LINE_AA)

        self.frame_number += 1
        result = {
            "frame": self.frame_number,
            "detections": detections,
            "inference_ms": round((time.time() - started) * 1000.0, 2),
            "stamp": message.header.stamp.to_sec(),
        }
        self.result_publisher.publish(String(data=json.dumps(result, ensure_ascii=True)))
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))
        if self.save_images and self.frame_number % self.save_interval == 0:
            filename = os.path.join(self.save_dir, "frame_{:06d}.jpg".format(self.frame_number))
            cv2.imwrite(filename, annotated)
        rospy.loginfo_throttle(
            5.0, "YOLO running: frame=%d detections=%d inference=%.1f ms",
            self.frame_number, len(detections), result["inference_ms"]
        )


if __name__ == "__main__":
    rospy.init_node("yolo_onnx_ros_test")
    YoloRosTest()
    rospy.spin()
