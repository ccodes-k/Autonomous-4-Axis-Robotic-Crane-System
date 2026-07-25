import ast

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from project_4.common import normalize_detected_box_name


class BoxCompareNode(Node):
    def __init__(self):
        super().__init__("box_compare_node")

        self.box_state = {
            "name": "NONE",
            "selected": 0,
        }
        self.zed_items = []

        self.pub = self.create_publisher(String, "/box_compare/result", 10)
        self.create_subscription(String, "/klipper/box_state", self.box_cb, 10)
        self.create_subscription(String, "/zed/detections", self.zed_cb, 10)

    def box_cb(self, msg: String):
        self.box_state = ast.literal_eval(msg.data)
        self.publish_result()

    def zed_cb(self, msg: String):
        self.zed_items = ast.literal_eval(msg.data)
        self.publish_result()

    def publish_result(self):
        selected = int(self.box_state.get("selected", 0))
        selected_name = str(self.box_state.get("name", "NONE"))

        result = {
            "box_match_ok": False,
            "box_match_msg": "NO_BOX_SELECTED",
            "matched_box_name": None,
        }

        if not selected or selected_name == "NONE":
            msg = String()
            msg.data = str(result)
            self.pub.publish(msg)
            return

        matched = False
        for det in self.zed_items:
            norm_name = normalize_detected_box_name(det["name"])
            if norm_name == selected_name:
                matched = True
                break

        if matched:
            result["box_match_ok"] = True
            result["box_match_msg"] = f"MATCHED {selected_name}"
            result["matched_box_name"] = selected_name
        else:
            result["box_match_msg"] = f"NO_MATCH_FOR {selected_name}"

        msg = String()
        msg.data = str(result)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoxCompareNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
