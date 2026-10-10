#!/usr/bin/env python3
import math
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Image


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class NavigationStatusWindow:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "smart_car")
        self.window_name = rospy.get_param("~window_name", "Camera Navigation")
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.width = int(rospy.get_param("~width", 760))
        self.height = int(rospy.get_param("~height", 620))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))

        self.bridge = CvBridge()
        self.last_image = None
        self.current_step = "waiting for mission_controller"
        self.last_capture = "-"
        self.get_model_state = None

        rospy.Subscriber(self.camera_topic, Image, self._on_image, queue_size=1)
        rospy.Subscriber("/rosout_agg", Log, self._on_log, queue_size=50)

    def _on_image(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Navigation window camera conversion failed: %s", exc)

    def _on_log(self, msg):
        text = msg.msg
        if "Mission step:" in text:
            self.current_step = text.split("Mission step:", 1)[1].strip()
        elif text.startswith("Capture ") or "Capture " in text:
            self.last_capture = text
        elif "Mission finished" in text:
            self.current_step = "Mission finished"

    def _connect_service(self):
        if self.get_model_state is not None:
            return True
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=0.5)
            self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            return True
        except Exception:
            return False

    def _read_pose(self):
        if not self._connect_service():
            return None
        try:
            state = self.get_model_state(self.model_name, "world")
        except Exception:
            self.get_model_state = None
            return None
        if not state.success:
            return None
        return state.pose

    def _draw_text(self, image, text, x, y, scale=0.55, color=(235, 235, 235), thickness=1):
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

    def _camera_panel(self, w, h):
        panel = np.zeros((h, w, 3), dtype=np.uint8)
        panel[:] = (18, 20, 24)
        if self.last_image is None:
            self._draw_text(panel, "waiting for camera: %s" % self.camera_topic, 24, h // 2, 0.62, (80, 210, 255), 2)
            return panel

        frame = self.last_image.copy()
        frame_h, frame_w = frame.shape[:2]
        scale = min(float(w) / frame_w, float(h) / frame_h)
        new_w = max(1, int(frame_w * scale))
        new_h = max(1, int(frame_h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
        x0 = (w - new_w) // 2
        y0 = (h - new_h) // 2
        panel[y0:y0 + new_h, x0:x0 + new_w] = resized
        return panel

    def _render(self):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:] = (28, 31, 36)
        header_h = 54
        footer_h = 86
        body_h = self.height - header_h - footer_h

        cv2.rectangle(image, (0, 0), (self.width, header_h), (36, 91, 150), -1)
        self._draw_text(image, "Realtime Camera Navigation", 20, 36, 0.78, (255, 255, 255), 2)

        camera = self._camera_panel(self.width, body_h)
        image[header_h:header_h + body_h, 0:self.width] = camera

        pose = self._read_pose()
        y0 = self.height - footer_h + 28
        if pose is None:
            pose_text = "pose: waiting for Gazebo model %s" % self.model_name
        else:
            yaw = math.degrees(yaw_from_quaternion(pose.orientation))
            pose_text = "pose: x=%.3f  y=%.3f  z=%.3f  yaw=%.1f deg" % (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                yaw,
            )
        self._draw_text(image, pose_text, 20, y0, 0.58, (240, 240, 240), 1)

        step = self.current_step
        if len(step) > 92:
            step = step[:89] + "..."
        self._draw_text(image, "step: " + step, 20, y0 + 26, 0.52, (220, 230, 245), 1)

        capture = self.last_capture
        if len(capture) > 92:
            capture = capture[:89] + "..."
        self._draw_text(image, "last capture: " + capture, 20, y0 + 52, 0.46, (175, 188, 205), 1)
        return image

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)

        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            cv2.imshow(self.window_name, self._render())
            if cv2.waitKey(1) == 27:
                rospy.signal_shutdown("sensor window closed by ESC")
                break
            rate.sleep()

        cv2.destroyWindow(self.window_name)


def main():
    rospy.init_node("navigation_status_window")
    NavigationStatusWindow().run()


if __name__ == "__main__":
    main()
