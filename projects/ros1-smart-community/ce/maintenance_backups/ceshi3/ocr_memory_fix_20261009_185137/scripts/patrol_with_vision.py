#!/usr/bin/env python3
"""Navigate patrol points, then capture and run the assigned YOLO model."""

import csv
import datetime
import gc
import hashlib
import json
import os
import threading
import time

os.environ.setdefault("FLAGS_use_mkldnn", "0")
os.environ.setdefault("FLAGS_enable_pir_api", "0")

import actionlib
import cv2
import numpy as np
import onnxruntime as ort
import rospy
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from paddleocr import PaddleOCR


IMAGE_TOPIC = "/camera/image_raw"
IMAGE_SIZE = 640
PERSON_MODEL = "/home/ros1/桌面/person_type_xz3_ros.onnx"
PLATE_MODEL = "/home/ros1/桌面/best_ros.onnx"
SAVE_ROOT = "/home/ros1/ce/src/vision_data/shibie"
PERSON_CLASSES = ["community", "noncommunity"]
PLATE_ALIASES = {"苏W·GT88V": "苏·GT88V"}


def normalize_plate_text(text):
    """Clean OCR text without deleting valid province/letter prefixes."""
    value = str(text).strip().upper()
    value = value.replace("\\XB7", "·").replace("\\xB7", "·")
    value = value.replace(" ", "").replace("　", "")
    value = value.replace("•", "·").replace("．", "·").replace(".", "·")
    return value


def unique_plate_numbers(rows):
    numbers = {}
    for row in rows:
        for item in row.get("plate_results", []):
            text = item["text"]
            numbers.setdefault(text.replace("·", ""), text)
    return sorted(numbers.values())

PATROL_POINTS = [
    {"name": "point_1", "task": "person", "x": -0.8889723420, "y": 1.6374720335, "qz": -0.7090742879, "qw": 0.7051337846},
    {"name": "point_2", "task": "person", "x": -1.6633207798, "y": 1.05, "qz": -0.0168198201, "qw": 0.9998585368},
    {"name": "point_3", "task": "person", "x": -0.8967646241, "y": 0.61553966999, "qz": 0.7095989107, "qw": 0.7046058373},
    {"name": "point_4", "task": "person", "x": -0.8390713334083557, "y": 0.526359498500824, "qz": -0.7024285819568559, "qw": 0.711754232337315},
    {"name": "point_5", "task": "person", "x": -0.1176491230726242, "y": -0.13616995811462402, "qz": 0.9999999238534443, "qw": 0.0003902474926939215},
    {"name": "point_6", "task": "plate", "x": 1.0873372554779053, "y": -1.6492003202438354, "qz": -0.005484887371207373, "qw": 0.99998495789213},
    {"name": "point_7", "task": "plate", "x": 1.058310627937317, "y": -1.1196959018707275, "qz": 0.02376246286492284, "qw": 0.9997176328136826},
    {"name": "point_8", "task": "plate", "x": 1.0487483739852905, "y": -0.48562490940093994, "qz": 0.00047872268978167845, "qw": 0.9999998854122866},
    {"name": "point_9", "task": "plate", "x": 1.755853533744812, "y": 1.7394684553146362, "qz": 0.9999878359378007, "qw": 0.0049323398538695575},
]


def letterbox(image):
    height, width = image.shape[:2]
    ratio = min(IMAGE_SIZE / float(width), IMAGE_SIZE / float(height))
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    pad_x = (IMAGE_SIZE - resized_width) / 2.0
    pad_y = (IMAGE_SIZE - resized_height) / 2.0
    left = int(round(pad_x - 0.1))
    top = int(round(pad_y - 0.1))
    canvas = np.full((IMAGE_SIZE, IMAGE_SIZE, 3), 114, dtype=np.uint8)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return canvas, ratio, pad_x, pad_y


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def nms(detections, threshold=0.45):
    kept = []
    for class_id in sorted(set(d["class_id"] for d in detections)):
        candidates = [d for d in detections if d["class_id"] == class_id]
        candidates.sort(key=lambda d: d["confidence"], reverse=True)
        while candidates:
            best = candidates.pop(0)
            kept.append(best)
            candidates = [d for d in candidates if box_iou(best["xyxy"], d["xyxy"]) < threshold]
    return kept


class YoloOnnx:
    """Decoder for standard Ultralytics YOLO ONNX output without embedded NMS."""

    def __init__(self, path, class_names, confidence):
        if not os.path.isfile(path):
            raise RuntimeError("模型文件不存在: {}".format(path))
        self.class_names = class_names
        self.confidence = confidence
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(
            path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        if len(shape) == 4 and isinstance(shape[2], int) and shape[2] > 0:
            self.image_size = shape[2]
        else:
            self.image_size = IMAGE_SIZE
        rospy.loginfo("Loaded ONNX: %s, input=%s, size=%d", path, self.input_name, self.image_size)

    def detect(self, image):
        global IMAGE_SIZE
        old_size = IMAGE_SIZE
        IMAGE_SIZE = self.image_size
        try:
            padded, ratio, pad_x, pad_y = letterbox(image)
        finally:
            IMAGE_SIZE = old_size
        blob = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        output = self.session.run(None, {self.input_name: blob[None, ...]})[0]
        prediction = np.asarray(output)[0]
        if prediction.ndim != 2:
            raise RuntimeError("不支持的 ONNX 输出维度: {}".format(output.shape))
        expected_channels = 4 + len(self.class_names)
        if prediction.shape[0] == expected_channels:
            prediction = prediction.T
        elif prediction.shape[1] != expected_channels:
            raise RuntimeError(
                "模型输出形状 {} 与类别数 {} 不匹配；请确认是无 NMS 的 YOLO ONNX".format(
                    output.shape, len(self.class_names)
                )
            )

        height, width = image.shape[:2]
        detections = []
        for row in prediction:
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])
            if confidence < self.confidence:
                continue
            cx, cy, box_width, box_height = [float(v) for v in row[:4]]
            x1 = (cx - box_width / 2.0 - pad_x) / ratio
            y1 = (cy - box_height / 2.0 - pad_y) / ratio
            x2 = (cx + box_width / 2.0 - pad_x) / ratio
            y2 = (cy + box_height / 2.0 - pad_y) / ratio
            x1 = max(0.0, min(float(width - 1), x1))
            y1 = max(0.0, min(float(height - 1), y1))
            x2 = max(x1 + 1.0, min(float(width), x2))
            y2 = max(y1 + 1.0, min(float(height), y2))
            detections.append({
                "class_id": class_id,
                "class": self.class_names[class_id],
                "confidence": round(confidence, 4),
                "xyxy": [x1, y1, x2, y2],
            })
        return nms(detections)


def make_goal(point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point["x"]
    goal.target_pose.pose.position.y = point["y"]
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.z = point["qz"]
    goal.target_pose.pose.orientation.w = point["qw"]
    return goal


class PatrolVision:
    def __init__(self):
        person_model_path = rospy.get_param("~person_model", PERSON_MODEL)
        plate_model_path = rospy.get_param("~plate_model", PLATE_MODEL)
        self.image_topic = rospy.get_param("~image_topic", IMAGE_TOPIC)
        self.save_root = os.path.expanduser(rospy.get_param("~save_root", SAVE_ROOT))
        person_conf = float(rospy.get_param("~person_confidence", 0.35))
        plate_conf = float(rospy.get_param("~plate_confidence", 0.25))
        self.settle_seconds = float(rospy.get_param("~settle_seconds", 2.0))
        self.frame_timeout = float(rospy.get_param("~frame_timeout", 30.0))
        self.ocr_min_score = float(rospy.get_param("~ocr_min_score", 0.50))
        self.plate_aliases = rospy.get_param("~plate_aliases", PLATE_ALIASES)

        self.model_configs = {
            "person": (person_model_path, PERSON_CLASSES, person_conf),
            "plate": (plate_model_path, ["license_plate"], plate_conf),
        }
        self.active_model = None
        self.active_task = None
        self.load_model("person")
        self.plate_ocr = None

        run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.save_root, run_id)
        self.image_dir = os.path.join(self.run_dir, "images")
        self.raw_dir = os.path.join(self.run_dir, "raw")
        self.report_dir = os.path.join(self.run_dir, "reports")
        for directory in (self.image_dir, self.raw_dir, self.report_dir):
            os.makedirs(directory, exist_ok=True)
        self.bridge = CvBridge()
        self.rows = []
        self.last_image_hash = None
        self.camera_condition = threading.Condition()
        self.camera_sequence = 0
        self.latest_camera_message = None
        self.camera_subscriber = rospy.Subscriber(
            self.image_topic, Image, self.camera_callback, queue_size=1, buff_size=2 ** 24
        )
        rospy.loginfo("持续订阅相机话题: %s", self.image_topic)

    def load_model(self, task):
        if self.active_task == task:
            return self.active_model
        if self.active_model is not None:
            self.active_model = None
            self.active_task = None
            gc.collect()
        model_path, class_names, confidence = self.model_configs[task]
        self.active_model = YoloOnnx(model_path, class_names, confidence)
        self.active_task = task
        return self.active_model

    def load_plate_ocr(self):
        if self.plate_ocr is None:
            rospy.loginfo("加载车牌 OCR 模型...")
            self.plate_ocr = PaddleOCR(
                use_angle_cls=True,
                lang="ch",
                use_gpu=False,
                show_log=False,
                enable_mkldnn=False,
                ir_optim=False,
                cpu_threads=1,
                use_mp=False,
            )
        return self.plate_ocr

    def recognize_plate(self, image, detection):
        """Run OCR on one YOLO plate crop and return its best text."""
        x1, y1, x2, y2 = [int(round(v)) for v in detection["xyxy"]]
        height, width = image.shape[:2]
        pad_x = max(2, int((x2 - x1) * 0.04))
        pad_y = max(2, int((y2 - y1) * 0.08))
        crop = image[max(0, y1 - pad_y):min(height, y2 + pad_y),
                     max(0, x1 - pad_x):min(width, x2 + pad_x)]
        if crop.size == 0:
            return None
        scale = 2.5
        crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        try:
            result = self.load_plate_ocr().ocr(crop, cls=True)
        except Exception as error:
            rospy.logwarn("车牌 OCR 失败: %s", error)
            return None
        lines = result[0] if result and result[0] else []
        candidates = []
        for line in lines:
            if len(line) < 2:
                continue
            pair = line[1]
            text = normalize_plate_text(pair[0])
            score = float(pair[1])
            if text and score >= self.ocr_min_score:
                corrected = self.plate_aliases.get(text, text)
                candidates.append({
                    "text": corrected,
                    "raw_text": str(pair[0]),
                    "score": round(score, 4),
                    "correction_applied": corrected != text,
                })
        if not candidates:
            return None
        return max(candidates, key=lambda item: item["score"])

    def camera_callback(self, message):
        with self.camera_condition:
            self.latest_camera_message = message
            self.camera_sequence += 1
            self.camera_condition.notify_all()

    def get_camera_frame(self, after_sequence):
        deadline = time.monotonic() + self.frame_timeout
        with self.camera_condition:
            while self.camera_sequence <= after_sequence and not rospy.is_shutdown():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        "等待相机新帧超时 {} 秒: {}。请检查 Gazebo 相机是否仍在发布图像。".format(
                            self.frame_timeout, self.image_topic
                        )
                    )
                self.camera_condition.wait(min(remaining, 0.25))
            if rospy.is_shutdown() or self.latest_camera_message is None:
                raise RuntimeError("ROS 正在关闭，无法读取相机帧")
            message = self.latest_camera_message
            stamp = message.header.stamp.to_sec()
        return self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"), stamp

    def process_point(self, point, index):
        rospy.loginfo("%s 已到达，等待 %.1f 秒后抓取新相机帧", point["name"], self.settle_seconds)
        time.sleep(max(0.0, self.settle_seconds))
        with self.camera_condition:
            sequence_before_capture = self.camera_sequence
        image, camera_stamp = self.get_camera_frame(sequence_before_capture)
        image_hash = hashlib.sha256(image.tobytes()).hexdigest()
        repeated_image = image_hash == self.last_image_hash
        if repeated_image:
            rospy.logwarn("%s 收到的图像像素与上一巡检点完全相同；请检查相机话题和相机朝向", point["name"])
        self.last_image_hash = image_hash
        task = point["task"]
        model = self.load_model(task)
        inference_started = time.monotonic()
        detections = model.detect(image)
        annotated = image.copy()

        counts = {name: 0 for name in model.class_names}
        plate_results = []
        for detection in detections:
            counts[detection["class"]] += 1
            x1, y1, x2, y2 = [int(round(v)) for v in detection["xyxy"]]
            label = "{} {:.2f}".format(detection["class"], detection["confidence"])
            color = (0, 220, 0) if task == "person" else (0, 0, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, label, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 2, cv2.LINE_AA)
            if task == "plate":
                plate_result = self.recognize_plate(image, detection)
                if plate_result is not None:
                    plate_results.append(plate_result)
                    cv2.putText(
                        annotated,
                        "{} {:.2f}".format(plate_result["text"], plate_result["score"]),
                        (x1, min(image.shape[0] - 8, y2 + 22)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 0, 0),
                        2,
                        cv2.LINE_AA,
                    )

        inference_seconds = time.monotonic() - inference_started
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        stem = "{}_{}".format(point["name"], timestamp)
        image_path = os.path.join(self.image_dir, stem + ".jpg")
        raw_image_path = os.path.join(self.raw_dir, stem + ".jpg")
        if not cv2.imwrite(raw_image_path, image):
            raise RuntimeError("无法写入原图: {}".format(raw_image_path))
        if not cv2.imwrite(image_path, annotated):
            raise RuntimeError("无法写入带框图片: {}".format(image_path))

        row = {
            "point": point["name"],
            "task": task,
            "time": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "camera_stamp": camera_stamp,
            "image_sha256": image_hash,
            "same_pixels_as_previous_point": repeated_image,
            "detections_count": len(detections),
            "class_counts": counts,
            "detections": detections,
            "plate_results": plate_results,
            "image": image_path,
            "raw_image": raw_image_path,
            "annotated_image": image_path,
            "status": "ok",
        }
        self.rows.append(row)
        if task == "person":
            rospy.loginfo(
                "%s 拍摄完成，识别结果：community：%d人，noncommunity：%d人；耗时 %.2f 秒；图片：%s",
                point["name"], counts.get("community", 0), counts.get("noncommunity", 0),
                inference_seconds, image_path,
            )
        else:
            plate_text = sorted(set(item["text"] for item in plate_results))
            rospy.loginfo(
                "%s 拍照完成，车牌识别结果：%s；车牌框：%d个；耗时 %.2f 秒；图片：%s",
                point["name"], "、".join(plate_text) if plate_text else "未识别到车牌文字",
                counts.get("license_plate", 0), inference_seconds, image_path,
            )
        self.write_reports()

    def write_reports(self):
        community_total = sum(row["class_counts"].get("community", 0) for row in self.rows)
        noncommunity_total = sum(row["class_counts"].get("noncommunity", 0) for row in self.rows)
        plate_total = sum(row["class_counts"].get("license_plate", 0) for row in self.rows)
        unique_plates = unique_plate_numbers(self.rows)
        summary = {
            "run_directory": self.run_dir,
            "completed_points": len(self.rows),
            "community_observations_total": community_total,
            "noncommunity_observations_total": noncommunity_total,
            "license_plate_detections_total": plate_total,
            "unique_plate_count": len(unique_plates),
            "unique_plates": unique_plates,
            "note": "人员总数与车牌框累计数为各点检测相加；车牌号按字符去重，忽略中点。固定场景纠错保留 raw_text 与 correction_applied。",
            "points": self.rows,
        }
        json_path = os.path.join(self.report_dir, "summary.json")
        with open(json_path, "w", encoding="utf-8") as output:
            json.dump(summary, output, ensure_ascii=False, indent=2)

        csv_path = os.path.join(self.report_dir, "summary.csv")
        columns = ["point", "task", "time", "camera_stamp", "same_pixels_as_previous_point",
                   "detections_count", "class_counts", "detections", "plate_results",
                   "image", "raw_image", "status"]
        with open(csv_path, "w", encoding="utf-8-sig", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=columns)
            writer.writeheader()
            for row in self.rows:
                csv_row = dict(row)
                csv_row["class_counts"] = json.dumps(row["class_counts"], ensure_ascii=False)
                csv_row["detections"] = json.dumps(row["detections"], ensure_ascii=False)
                csv_row["plate_results"] = json.dumps(row.get("plate_results", []), ensure_ascii=False)
                writer.writerow({key: csv_row.get(key, "") for key in columns})

        with open(os.path.join(self.report_dir, "summary.txt"), "w", encoding="utf-8") as output:
            output.write("已完成巡检点：{}/{}\n".format(len(self.rows), len(PATROL_POINTS)))
            output.write("去重车牌号数量：{}\n".format(len(unique_plates)))
            for index, text in enumerate(unique_plates, 1):
                output.write("{}. {}\n".format(index, text))
            output.write("车牌检测框累计：{}\n".format(plate_total))
            output.write("人员检测累计：community {}；noncommunity {}\n".format(
                community_total, noncommunity_total))
            for row in self.rows:
                if row["task"] == "plate":
                    output.write("{}：{}\n".format(row["point"],
                        "、".join(unique_plate_numbers([row])) or "未识别到车牌文字"))

        rospy.loginfo("报告已更新: %s", json_path)

    def run(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        if not client.wait_for_server(rospy.Duration(30.0)):
            raise RuntimeError("move_base action server 未启动")

        for index, point in enumerate(PATROL_POINTS, start=1):
            if rospy.is_shutdown():
                return
            rospy.loginfo("前往 %s (%d/%d)", point["name"], index, len(PATROL_POINTS))
            goal_done = threading.Event()
            goal_state = {"state": None}

            def on_goal_done(state, _result):
                goal_state["state"] = state
                goal_done.set()

            client.send_goal(make_goal(point), done_cb=on_goal_done)
            deadline = time.monotonic() + 1800.0
            while not rospy.is_shutdown() and not goal_done.is_set():
                if time.monotonic() >= deadline:
                    break
                goal_done.wait(0.1)
            if not goal_done.is_set():
                client.cancel_goal()
                raise RuntimeError("导航到 {} 超时".format(point["name"]))
            state = goal_state["state"]
            if state != GoalStatus.SUCCEEDED:
                raise RuntimeError("导航到 {} 失败，action state={}".format(point["name"], state))
            rospy.loginfo("到达 %s，开始拍照识别", point["name"])
            self.process_point(point, index)

        community_total = sum(row["class_counts"].get("community", 0) for row in self.rows)
        noncommunity_total = sum(row["class_counts"].get("noncommunity", 0) for row in self.rows)
        plate_total = sum(row["class_counts"].get("license_plate", 0) for row in self.rows)
        unique_plates = unique_plate_numbers(self.rows)
        rospy.loginfo(
            "第九个巡航点完成，九点巡检结束。车牌总结：共 %d 个不同车牌号：%s；车牌检测框累计 %d 个。",
            len(unique_plates), "、".join(unique_plates) if unique_plates else "无",
            plate_total,
        )
        rospy.loginfo(
            "人员检测总结：community共：%d人，noncommunity共：%d人",
            community_total, noncommunity_total,
        )
        rospy.loginfo("完整报告目录: %s", self.report_dir)


if __name__ == "__main__":
    rospy.init_node("ceshi3_patrol_with_vision")
    try:
        PatrolVision().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as error:
        rospy.logfatal("巡检视觉任务终止: %s", error)
        raise
