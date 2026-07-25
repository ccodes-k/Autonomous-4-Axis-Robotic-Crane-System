import ast
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")

        self.zed_items = []
        self.box_state = {
            "name": "NONE",
            "length": 0.0,
            "width": 0.0,
            "height": 0.0,
            "selected": 0,
        }
        self.compare_state = {
            "box_match_ok": False,
            "box_match_msg": "NO_BOX_SELECTED",
            "matched_box_name": None,
            "matched_box_length": None,
            "matched_box_width": None,
            "matched_box_height": None,
        }
        self.safety_state = {
            "status": "NO_PERSON",
            "distance_m": None,
            "hits": 0,
            "estop_sent": False,
            "last_msg": "",
        }

        self.last_frame = None
        self.first_draw = True
        self.tty = None

        self.create_subscription(String, "/zed/detections", self.zed_cb, 10)
        self.create_subscription(String, "/klipper/box_state", self.box_cb, 10)
        self.create_subscription(String, "/box_compare/result", self.compare_cb, 10)
        self.create_subscription(String, "/safety_camera/status", self.safety_cb, 10)

        self.open_tty()
        self.timer = self.create_timer(0.2, self.render)

    def open_tty(self):
        try:
            # bypass ros2 launch stdout capture/prefixing
            self.tty = open("/dev/tty", "w", buffering=1)
            # clear once
            self.tty.write("\x1b[2J\x1b[H")
            self.tty.flush()
        except Exception as e:
            self.tty = None
            self.get_logger().error(f"Could not open /dev/tty: {e}")

    def zed_cb(self, msg: String):
        self.zed_items = ast.literal_eval(msg.data)

    def box_cb(self, msg: String):
        self.box_state = ast.literal_eval(msg.data)

    def compare_cb(self, msg: String):
        self.compare_state = ast.literal_eval(msg.data)

    def safety_cb(self, msg: String):
        self.safety_state = ast.literal_eval(msg.data)

    def short_box_line(self):
        if self.box_state.get("selected", 0):
            return f"BOX: {self.box_state.get('name', 'NONE')}"
        return "BOX: NONE"

    def short_zed_line(self):
        if not self.zed_items:
            return "ZED: none"

        det = self.zed_items[0]
        name = det.get("name", "unknown")
        dist = det.get("distance_m", None)
        if dist is None:
            return f"ZED: {name}"
        return f"ZED: {name} {dist:.3f} m"

    def short_compare_line(self):
        return f"CMP: {self.compare_state.get('box_match_msg', 'NO_BOX_SELECTED')}"

    def short_safety_line(self):
        status = self.safety_state.get("status", "NO_PERSON")
        dist = self.safety_state.get("distance_m", None)
        hits = int(self.safety_state.get("hits", 0))
        estop = self.safety_state.get("estop_sent", False)

        if dist is None:
            return f"SAFE: {status} | hits={hits} | estop={estop}"

        return f"SAFE: {status} {dist:.3f} m | hits={hits} | estop={estop}"

    def short_msg_line(self):
        msg = self.safety_state.get("last_msg", "")
        if not msg:
            msg = "-"
        if len(msg) > 80:
            msg = msg[:77] + "..."
        return f"MSG: {msg}"

    def build_frame(self):
        return [
            "=== CAMERA MONITOR ===",
            self.short_box_line(),
            self.short_zed_line(),
            self.short_compare_line(),
            self.short_safety_line(),
            self.short_msg_line(),
        ]

    def draw_frame(self, lines):
        if self.tty is None:
            return

        width = 120

        if self.first_draw:
            for line in lines:
                self.tty.write(line.ljust(width) + "\n")
            self.tty.flush()
            self.first_draw = False
            return

        # move cursor up by number of monitor lines and redraw in place
        self.tty.write(f"\x1b[{len(lines)}F")
        for line in lines:
            self.tty.write("\r" + line.ljust(width) + "\n")
        self.tty.flush()

    def render(self):
        lines = self.build_frame()

        # only redraw if content changed
        if lines != self.last_frame:
            self.draw_frame(lines)
            self.last_frame = lines

    def destroy_node(self):
        try:
            if self.tty is not None:
                self.tty.write("\n")
                self.tty.flush()
                self.tty.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
