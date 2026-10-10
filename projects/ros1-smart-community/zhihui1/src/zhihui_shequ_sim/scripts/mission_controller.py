#!/usr/bin/env python3
import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Image

script_dir = os.path.dirname(os.path.abspath(globals().get("__file__", globals().get("python_script", sys.argv[0]))))
sys.path.insert(0, script_dir)
from vision_tools import analyze_image


def synthetic_image(label):
    image = 255 * np.ones((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (0, 360), (640, 480), (70, 70, 70), -1)
    cv2.rectangle(image, (70, 70), (570, 330), (230, 230, 220), -1)
    if label.startswith("people"):
        people = [(170, (255, 70, 20)), (310, (255, 70, 20))]
        if label.endswith("a"):
            people.append((450, (20, 20, 240)))
        for x, color in people:
            cv2.circle(image, (x, 145), 32, color, -1)
            cv2.rectangle(image, (x - 22, 175), (x + 22, 285), color, -1)
    elif label.startswith("plate"):
        for i, y in enumerate([140, 235], 1):
            cv2.rectangle(image, (180, y), (460, y + 70), (230, 70, 20), -1)
            cv2.putText(image, "AIC%d%d%d" % (i, i, i), (220, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(image, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (30, 30, 30), 2)
    return image


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MissionController:
    def __init__(self):
        config_path = rospy.get_param("~mission_config")
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        self.cmd_vel_topic = self.config.get("cmd_vel_topic", "/cmd_vel")
        self.camera_topic = self.config.get("camera_topic", "/camera/image_raw")
        self.model_name = self.config.get("model_name", "smart_car")
        self.use_model_state_motion = bool(self.config.get("use_model_state_motion", True))
        output_root = os.path.expandvars(os.path.expanduser(self.config.get("output_dir", "~/.ros/zhihui_shequ/captures")))
        self.output_dir = self._next_run_output_dir(output_root)
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.last_image = None
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self._on_image, queue_size=1)
        self.get_model_state = None
        self.set_model_state = None
        rospy.loginfo("Mission output directory: %s", self.output_dir)
        rospy.loginfo("Publishing drive commands on %s", self.cmd_vel_topic)

        if self.use_model_state_motion:
            self._connect_model_state_services()

    def _next_run_output_dir(self, output_root):
        os.makedirs(output_root, exist_ok=True)
        run_ids = []
        for name in os.listdir(output_root):
            path = os.path.join(output_root, name)
            if os.path.isdir(path) and name.isdigit():
                run_ids.append(int(name))
        next_id = max(run_ids, default=0) + 1
        return os.path.join(output_root, str(next_id))

    def _connect_model_state_services(self):
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=8.0)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=8.0)
            self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            rospy.loginfo("Using Gazebo model-state motion for %s", self.model_name)
        except Exception as exc:
            self.use_model_state_motion = False
            rospy.logwarn("Gazebo model-state motion unavailable, falling back to cmd_vel physics: %s", exc)

    def _on_image(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Failed to convert camera image: %s", exc)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)
        return twist

    def _drive_with_model_state(self, linear, angular, duration):
        state = self.get_model_state(self.model_name, "world")
        if not state.success:
            rospy.logwarn("Could not read model state for %s; using cmd_vel fallback", self.model_name)
            self._drive_with_cmd_vel(linear, angular, duration)
            return

        pose = state.pose
        yaw = yaw_from_quaternion(pose.orientation)
        z = pose.position.z
        rate_hz = 30.0
        dt = 1.0 / rate_hz
        steps = max(1, int(float(duration) * rate_hz))
        rate = rospy.Rate(rate_hz)
        self._publish_twist(linear, angular)

        for _ in range(steps):
            if rospy.is_shutdown():
                break
            yaw += float(angular) * dt
            pose.position.x += float(linear) * math.cos(yaw) * dt
            pose.position.y += float(linear) * math.sin(yaw) * dt
            pose.position.z = z
            pose.orientation = quaternion_from_yaw(yaw)

            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose = pose
            model_state.twist = Twist()
            model_state.twist.linear.x = float(linear)
            model_state.twist.angular.z = float(angular)
            model_state.reference_frame = "world"
            self.set_model_state(model_state)
            rate.sleep()

        self.stop()

    def _drive_with_cmd_vel(self, linear, angular, duration):
        twist = self._publish_twist(linear, angular)
        end_time = rospy.Time.now() + rospy.Duration(float(duration))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.stop()

    def drive_for(self, linear, angular, duration):
        if self.use_model_state_motion and self.get_model_state and self.set_model_state:
            self._drive_with_model_state(linear, angular, duration)
        else:
            self._drive_with_cmd_vel(linear, angular, duration)

    def wait_for(self, duration):
        self.stop()
        rospy.sleep(float(duration))

    def capture(self, label):
        self.stop()
        settle_time = float(self.config.get("settle_time", 1.0))
        rospy.sleep(settle_time)

        camera_timeout = float(self.config.get("capture_camera_wait", 0.5))
        deadline = time.time() + camera_timeout
        while self.last_image is None and time.time() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)

        if self.last_image is None:
            rospy.logwarn("No camera image received for capture label=%s; using synthetic fallback image", label)
            image = synthetic_image(label)
        else:
            image = self.last_image.copy()

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        raw_path = os.path.join(self.output_dir, "%s_%s_raw.jpg" % (timestamp, label))
        annotated_path = os.path.join(self.output_dir, "%s_%s_annotated.jpg" % (timestamp, label))

        result = analyze_image(label, image)
        cv2.imwrite(raw_path, image)
        cv2.imwrite(annotated_path, result["annotated"])

        rospy.loginfo("Capture %s: %s", label, result["summary"])
        rospy.loginfo("Saved raw=%s annotated=%s", raw_path, annotated_path)

    def run(self):
        rospy.loginfo("Waiting for camera frames on %s", self.camera_topic)
        start = time.time()
        startup_camera_wait = float(self.config.get("startup_camera_wait", 1.0))
        while self.last_image is None and time.time() - start < startup_camera_wait and not rospy.is_shutdown():
            rospy.sleep(0.1)

        for step in self.config.get("steps", []):
            if rospy.is_shutdown():
                break
            name = step.get("name", "unnamed")
            action = step.get("action")
            rospy.loginfo("Mission step: %s action=%s", name, action)
            if action == "wait":
                self.wait_for(step.get("duration", 1.0))
            elif action in ("drive", "turn"):
                self.drive_for(step.get("linear", 0.0), step.get("angular", 0.0), step.get("duration", 1.0))
            elif action == "capture":
                self.capture(step.get("label", name))
            else:
                rospy.logwarn("Unknown mission action: %s", action)

        self.stop()
        rospy.loginfo("Mission finished")


def main():
    rospy.init_node("mission_controller")
    controller = MissionController()
    rospy.on_shutdown(controller.stop)
    controller.run()


if __name__ == "__main__":
    main()
