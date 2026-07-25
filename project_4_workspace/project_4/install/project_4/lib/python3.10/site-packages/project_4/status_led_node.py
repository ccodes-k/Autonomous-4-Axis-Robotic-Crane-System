import ast
import json
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


class StatusLedNode(Node):
    def __init__(self):
        super().__init__("status_led_node")

        self.declare_parameter("green_gpio", 4)
        self.declare_parameter("red_gpio", 22)
        self.declare_parameter("moonraker_base_url", "http://192.168.1.210:7125")
        self.declare_parameter("moonraker_api_key", "")
        self.declare_parameter("near_status_values", ["DANGER", "STOP"])
        self.declare_parameter("poll_interval", 0.5)

        self.green_gpio = self.get_parameter("green_gpio").value
        self.red_gpio = self.get_parameter("red_gpio").value
        self.base_url = self.get_parameter("moonraker_base_url").value
        self.api_key = self.get_parameter("moonraker_api_key").value
        self.near_status_values = set(self.get_parameter("near_status_values").value)

        self.person_near = False
        self.machine_running = False

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.green_gpio, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.red_gpio, GPIO.OUT, initial=GPIO.LOW)

        self.create_subscription(String, "/safety_camera/status", self.safety_cb, 10)
        self.timer = self.create_timer(
            float(self.get_parameter("poll_interval").value),
            self.poll_and_update,
        )

        self.get_logger().info(
            f"LED node started: green GPIO {self.green_gpio}, red GPIO {self.red_gpio}"
        )

    def safety_cb(self, msg: String):
        try:
            data = ast.literal_eval(msg.data)
            status = str(data.get("status", "NO_PERSON"))
            self.person_near = status in self.near_status_values
        except Exception as e:
            self.get_logger().warn(f"Bad /safety_camera/status payload: {e}")

    def moonraker_headers(self):
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["X-Api-Key"] = self.api_key
        return headers

    def query_print_state(self):
        url = f"{self.base_url.rstrip('/')}/printer/objects/query"
        payload = {"objects": {"print_stats": None}}

        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers=self.moonraker_headers(),
            method="POST",
        )

        with urllib.request.urlopen(req, timeout=2.0) as resp:
            raw = resp.read().decode("utf-8", errors="ignore")
            data = json.loads(raw)

        state = (
            data.get("result", {})
            .get("status", {})
            .get("print_stats", {})
            .get("state", "")
        )

        return str(state).lower()

    def poll_and_update(self):
        try:
            state = self.query_print_state()
            self.machine_running = state == "printing"
        except Exception as e:
            self.machine_running = False
            self.get_logger().warn(f"Moonraker query failed: {e}")

        GPIO.output(self.red_gpio, GPIO.HIGH if self.person_near else GPIO.LOW)
        GPIO.output(self.green_gpio, GPIO.HIGH if self.machine_running else GPIO.LOW)

    def destroy_node(self):
        try:
            GPIO.output(self.red_gpio, GPIO.LOW)
            GPIO.output(self.green_gpio, GPIO.LOW)
            GPIO.cleanup([self.red_gpio, self.green_gpio])
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusLedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
