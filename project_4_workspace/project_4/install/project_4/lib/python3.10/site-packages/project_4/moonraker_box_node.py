import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from project_4.common import query_box_state


class MoonrakerBoxNode(Node):
    def __init__(self):
        super().__init__("moonraker_box_node")

        self.declare_parameter("moonraker_base_url", "http://192.168.1.210:7125")
        self.declare_parameter("moonraker_api_key", "")
        self.declare_parameter("poll_interval", 0.5)

        self.base_url = self.get_parameter("moonraker_base_url").value
        self.api_key = self.get_parameter("moonraker_api_key").value or None
        self.poll_interval = float(self.get_parameter("poll_interval").value)

        self.pub = self.create_publisher(String, "/klipper/box_state", 10)
        self.timer = self.create_timer(self.poll_interval, self.loop)

    def loop(self):
        try:
            box = query_box_state(self.base_url, self.api_key)
        except Exception as e:
            box = {
                "name": "NONE",
                "length": 0.0,
                "width": 0.0,
                "height": 0.0,
                "selected": 0,
                "error": str(e),
            }

        msg = String()
        msg.data = str(box)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoonrakerBoxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
