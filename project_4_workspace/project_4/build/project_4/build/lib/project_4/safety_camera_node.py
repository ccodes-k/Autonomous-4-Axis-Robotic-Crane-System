import ast
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

from project_4.common import estimate_distance_m, send_safety_pause, query_safety_state



class SafetyCameraNode(Node):
    def __init__(self):
        super().__init__("safety_camera_node")

        self.declare_parameter("model_path", "/home/ros2/yolo11n.pt")
        self.declare_parameter("calib_path", "/home/ros2/c910_calibration.npz")
        self.declare_parameter("cam_index", 0)
        self.declare_parameter("conf_thresh", 0.45)
        self.declare_parameter("img_size", 640)
        self.declare_parameter("real_shoulder_width_m", 0.42)
        self.declare_parameter("min_bbox_width_px", 40)
        self.declare_parameter("smooth_window", 5)
        self.declare_parameter("stop_distance_m", 1.20)
        self.declare_parameter("estop_cooldown_s", 5.0)
        self.declare_parameter("require_hits", 3)
        self.declare_parameter("moonraker_base_url", "http://192.168.1.210:7125")
        self.declare_parameter("moonraker_api_key", "")
        self.declare_parameter("resume_poll_interval_s", 2.0)  # NEW

        self.model_path = self.get_parameter("model_path").value
        self.calib_path = self.get_parameter("calib_path").value
        self.cam_index = int(self.get_parameter("cam_index").value)
        self.conf_thresh = float(self.get_parameter("conf_thresh").value)
        self.img_size = int(self.get_parameter("img_size").value)
        self.real_width = float(self.get_parameter("real_shoulder_width_m").value)
        self.min_bbox_width = int(self.get_parameter("min_bbox_width_px").value)
        self.smooth_window = int(self.get_parameter("smooth_window").value)
        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.estop_cooldown = float(self.get_parameter("estop_cooldown_s").value)
        self.require_hits = int(self.get_parameter("require_hits").value)
        self.base_url = self.get_parameter("moonraker_base_url").value
        self.api_key = self.get_parameter("moonraker_api_key").value or None
        self.resume_poll_interval = float(
            self.get_parameter("resume_poll_interval_s").value
        )

        self.pub = self.create_publisher(String, "/safety_camera/status", 10)

        if not os.path.exists(self.calib_path):
            raise FileNotFoundError(self.calib_path)

        calib = np.load(self.calib_path)
        self.camera_matrix = calib["camera_matrix"]
        self.dist_coeffs = calib["dist_coeffs"]
        self.fx = float(self.camera_matrix[0, 0])

        self.model = YOLO(self.model_path)
        self.cap = cv2.VideoCapture(self.cam_index)
        if not self.cap.isOpened():
            raise RuntimeError("Safety camera failed to open")

        self.distance_history = deque(maxlen=self.smooth_window)
        self.below_threshold_count = 0
        self.last_estop_time = 0.0

        # --- NEW pause-state tracking ---
        # True while we are waiting for Klipper to leave the paused state
        self.paused_by_us = False
        self.last_resume_poll_time = 0.0
        self.last_estop_msg = ""

        self.timer = self.create_timer(0.1, self.loop)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_printer_state(self) -> str | None:
        try:
            return query_safety_state(self.base_url, self.api_key)
        except Exception as e:
            self.get_logger().warn(f"State poll failed: {e}")
            return None
    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def loop(self):
        now = time.time()

        # ── If we issued a pause, block detection and wait for Klipper
        #    to transition OUT of "paused" before we care again. ────────
        if self.paused_by_us:
            if (now - self.last_resume_poll_time) >= self.resume_poll_interval:
                self.last_resume_poll_time = now
                state = self._get_printer_state()
                if state is not None and state != "paused":
                    self.get_logger().info(
                        f"Klipper left paused state ({state}); safety re-armed."
                    )
                    self.paused_by_us = False
                    self.below_threshold_count = 0
                    self.distance_history.clear()
                    self.last_estop_msg = f"RESUMED detected state={state}"
                else:
                    # Still paused — publish holding status and return
                    self._publish(
                        {
                            "status": "PAUSED_WAITING_RESUME",
                            "distance_m": None,
                            "hits": 0,
                            "paused_by_us": True,
                            "last_msg": self.last_estop_msg,
                        }
                    )
                    return

        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(
            frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix
        )

        results = self.model.predict(
            source=undistorted,
            imgsz=self.img_size,
            conf=self.conf_thresh,
            verbose=False,
        )

        closest_dist = None
        for r in results:
            if r.boxes is None:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            classes = r.boxes.cls.cpu().numpy()
            for box, cls_id in zip(boxes, classes):
                if int(cls_id) != 0:
                    continue
                x1, y1, x2, y2 = map(int, box)
                bbox_w = x2 - x1
                if bbox_w < self.min_bbox_width:
                    continue
                dist_m = estimate_distance_m(bbox_w, self.real_width, self.fx)
                if dist_m is None:
                    continue
                if closest_dist is None or dist_m < closest_dist:
                    closest_dist = dist_m

        status_dict = {
            "status": "NO_PERSON",
            "distance_m": None,
            "hits": 0,
            "paused_by_us": False,
            "last_msg": self.last_estop_msg,
        }

        if closest_dist is not None:
            self.distance_history.append(closest_dist)
            smooth_dist = sum(self.distance_history) / len(self.distance_history)

            if smooth_dist <= self.stop_distance:
                self.below_threshold_count += 1
                state = "DANGER"
            else:
                self.below_threshold_count = 0
                state = "SAFE"

            # ── Trigger pause ─────────────────────────────────────────
            if (
                smooth_dist <= self.stop_distance
                and self.below_threshold_count >= self.require_hits
                and (now - self.last_estop_time) >= self.estop_cooldown
            ):
                try:
                    http_status, _ = send_safety_pause(self.base_url, self.api_key)
                    self.paused_by_us = True          # arm the resume-watcher
                    self.last_estop_time = now
                    self.last_resume_poll_time = now
                    self.last_estop_msg = f"PAUSE_SENT HTTP {http_status}"
                    self.get_logger().warn(
                        f"Safety pause issued (dist={smooth_dist:.2f} m)."
                    )
                except Exception as e:
                    self.last_estop_time = now
                    self.last_estop_msg = f"PAUSE_FAIL {e}"

            status_dict = {
                "status": state,
                "distance_m": smooth_dist,
                "hits": self.below_threshold_count,
                "paused_by_us": self.paused_by_us,
                "last_msg": self.last_estop_msg,
            }
        else:
            self.below_threshold_count = 0

        self._publish(status_dict)

    def _publish(self, d: dict):
        msg = String()
        msg.data = str(d)
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
