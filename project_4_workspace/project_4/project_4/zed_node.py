import math

import cv2
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

from project_4.common import clamp, median_xyz


class ZedNode(Node):
    def __init__(self):
        super().__init__("zed_node")

        self.declare_parameter("model_path", "/home/ros2/runs/box_detector11/weights/best.pt")
        self.declare_parameter("conf_thresh", 0.30)

        self.model_path = self.get_parameter("model_path").value
        self.conf_thresh = float(self.get_parameter("conf_thresh").value)

        self.pub = self.create_publisher(String, "/zed/detections", 10)

        self.model = YOLO(self.model_path)

        self.zed = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 30
        init.depth_mode = sl.DEPTH_MODE.ULTRA
        init.coordinate_units = sl.UNIT.METER

        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED_OPEN_FAILED: {status}")
            raise RuntimeError(f"ZED open failed: {status}")

        self.runtime = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.point_cloud = sl.Mat()

        self.timer = self.create_timer(0.2, self.loop)

    def loop(self):
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)

        frame = self.image.get_data()
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        results = self.model.predict(frame, conf=self.conf_thresh, verbose=False)

        detections = []
        if len(results) > 0 and results[0].boxes is not None:
            r = results[0]
            names = r.names
            h, w = frame.shape[:2]

            for box in r.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                cls_name = names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                x1 = clamp(x1, 0, w - 1)
                x2 = clamp(x2, 0, w - 1)
                y1 = clamp(y1, 0, h - 1)
                y2 = clamp(y2, 0, h - 1)

                if x2 <= x1 or y2 <= y1:
                    continue

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                center_3d = median_xyz(self.point_cloud, cx, cy)

                distance_m = None
                if center_3d is not None:
                    distance_m = math.sqrt(
                        center_3d[0] ** 2 + center_3d[1] ** 2 + center_3d[2] ** 2
                    )

                if distance_m is not None:
                    detections.append(
                        {
                            "name": cls_name,
                            "conf": conf,
                            "distance_m": distance_m,
                        }
                    )

        detections.sort(key=lambda x: x["distance_m"])
        msg = String()
        msg.data = str(detections[:5])
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.zed.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
