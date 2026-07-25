import ast
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from project_4.motion_api import send_gcode


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__("mission_controller_node")

        self.declare_parameter("moonraker_base_url", "http://192.168.1.210:7125")
        self.declare_parameter("moonraker_api_key", "")
        self.declare_parameter("auto_start", True)

        self.declare_parameter("loop_period_s", 0.2)
        self.declare_parameter("screen_idle_timeout_s", 10.0)
        self.declare_parameter("command_wait_s", 1.0)
        self.declare_parameter("place_wait_s", 1.5)
        self.declare_parameter("base_scan_timeout_s", 5.0)

        self.declare_parameter("base_fill_length_mm", 600.0)
        self.declare_parameter("fill_dimension", "length")   # length or width
        self.declare_parameter("y_step_default_mm", 20.0)    # must match Y_FORWARD default in Klipper

        self.declare_parameter("base_class_names", ["base"])

        self.base_url = self.get_parameter("moonraker_base_url").value
        self.api_key = self.get_parameter("moonraker_api_key").value or None
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.loop_period_s = float(self.get_parameter("loop_period_s").value)
        self.screen_idle_timeout_s = float(self.get_parameter("screen_idle_timeout_s").value)
        self.command_wait_s = float(self.get_parameter("command_wait_s").value)
        self.place_wait_s = float(self.get_parameter("place_wait_s").value)
        self.base_scan_timeout_s = float(self.get_parameter("base_scan_timeout_s").value)

        self.base_fill_length_mm = float(self.get_parameter("base_fill_length_mm").value)
        self.fill_dimension = str(self.get_parameter("fill_dimension").value).strip().lower()
        self.y_step_default_mm = float(self.get_parameter("y_step_default_mm").value)

        raw_names = self.get_parameter("base_class_names").value
        self.base_class_names = {str(x).strip().lower() for x in raw_names}

        self.box_state = {
            "name": "NONE",
            "length": 0.0,
            "width": 0.0,
            "height": 0.0,
            "selected": 0,
        }
        self.zed_items = []
        self.compare_state = {
            "box_match_ok": False,
            "box_match_msg": "NO_BOX_SELECTED",
        }
        self.safety_state = {
            "status": "NO_PERSON",
        }

        self.state = "IDLE"
        self.state_once = False
        self.wait_until = 0.0
        self.scan_deadline = 0.0

        self.active_box = None
        self.placed_count = 0
        self.slot_offset_y_mm = 0.0

        self.last_box_signature = None
        self.selection_event_counter = 0
        self.handled_selection_event = 0

        self.last_activity_ts = time.monotonic()
        self.screen_awake = True
        self.last_hold_state = None
        self.last_debug = None

        self.pub_state = self.create_publisher(String, "/mission/state", 10)
        self.pub_debug = self.create_publisher(String, "/mission/debug", 10)

        self.create_subscription(String, "/klipper/box_state", self.box_cb, 10)
        self.create_subscription(String, "/zed/detections", self.zed_cb, 10)
        self.create_subscription(String, "/box_compare/result", self.compare_cb, 10)
        self.create_subscription(String, "/safety_camera/status", self.safety_cb, 10)

        self.timer = self.create_timer(self.loop_period_s, self.loop)

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------
    def box_cb(self, msg: String):
        try:
            data = ast.literal_eval(msg.data)
            if isinstance(data, dict):
                self.box_state = data
        except Exception as e:
            self.get_logger().warn(f"Bad /klipper/box_state payload: {e}")
            return

        sig = (
            int(self.box_state.get("selected", 0)),
            str(self.box_state.get("name", "NONE")),
        )
        if sig != self.last_box_signature:
            self.last_box_signature = sig
            self.last_activity_ts = time.monotonic()
            self._screen_on_optional()

            if sig[0] == 1 and sig[1] != "NONE":
                self.selection_event_counter += 1

    def zed_cb(self, msg: String):
        try:
            data = ast.literal_eval(msg.data)
            self.zed_items = data if isinstance(data, list) else []
        except Exception as e:
            self.get_logger().warn(f"Bad /zed/detections payload: {e}")

    def compare_cb(self, msg: String):
        try:
            data = ast.literal_eval(msg.data)
            if isinstance(data, dict):
                self.compare_state = data
        except Exception as e:
            self.get_logger().warn(f"Bad /box_compare/result payload: {e}")

    def safety_cb(self, msg: String):
        try:
            data = ast.literal_eval(msg.data)
            if isinstance(data, dict):
                self.safety_state = data
        except Exception as e:
            self.get_logger().warn(f"Bad /safety_camera/status payload: {e}")

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------
    def _now(self) -> float:
        return time.monotonic()

    def _start_wait(self, seconds: float):
        self.wait_until = self._now() + max(0.0, seconds)

    def _wait_done(self) -> bool:
        return self._now() >= self.wait_until

    def publish_debug(self, text: str):
        if text == self.last_debug:
            return
        self.last_debug = text
        msg = String()
        msg.data = text
        self.pub_debug.publish(msg)
        self.get_logger().info(text)

    def publish_state(self, extra=None):
        payload = {
            "state": self.state,
            "selected_box": self.box_state.get("name", "NONE"),
            "placed_count": self.placed_count,
            "slot_offset_y_mm": round(self.slot_offset_y_mm, 3),
            "screen_awake": self.screen_awake,
        }
        if self.active_box is not None:
            payload["active_box"] = self.active_box
        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload)
        self.pub_state.publish(msg)

    def set_state(self, new_state: str):
        if new_state != self.state:
            self.state = new_state
            self.state_once = False
            self.publish_state()

    def _send_macro(self, macro: str, optional: bool = False) -> bool:
        try:
            send_gcode(self.base_url, macro, api_key=self.api_key)
            return True
        except Exception as e:
            if optional:
                self.get_logger().warn(f"Optional macro failed: {macro}: {e}")
            else:
                self.publish_debug(f"MACRO_FAIL {macro}: {e}")
            return False

    def _screen_on_optional(self):
        if self.screen_awake:
            return
        self._send_macro("SCREEN_ON", optional=True)
        self.screen_awake = True

    def _screen_off_optional(self):
        if not self.screen_awake:
            return
        self._send_macro("SCREEN_OFF", optional=True)
        self.screen_awake = False

    def _handle_screen_timeout(self):
        if self.state != "IDLE":
            self._screen_on_optional()
            return

        if (self._now() - self.last_activity_ts) >= self.screen_idle_timeout_s:
            self._screen_off_optional()

    def _safety_hold(self) -> bool:
        status = str(self.safety_state.get("status", "NO_PERSON"))
        if status in ("DANGER", "PAUSED_WAITING_RESUME"):
            if self.last_hold_state != status:
                self.last_hold_state = status
                self.publish_debug(f"MISSION_HOLD {status}")
            return True

        if self.last_hold_state is not None:
            self.publish_debug("MISSION_RESUME")
            self.last_hold_state = None

        return False

    def _selected_box_ready(self) -> bool:
        return (
            int(self.box_state.get("selected", 0)) == 1
            and str(self.box_state.get("name", "NONE")) != "NONE"
        )

    def _capture_active_box(self):
        self.active_box = {
            "name": str(self.box_state.get("name", "NONE")),
            "length": float(self.box_state.get("length", 0.0)),
            "width": float(self.box_state.get("width", 0.0)),
            "height": float(self.box_state.get("height", 0.0)),
        }

    def _base_seen(self) -> bool:
        for det in self.zed_items:
            name = str(det.get("name", "")).strip().lower()
            if name in self.base_class_names:
                return True
        return False

    def _box_match_ok(self) -> bool:
        return bool(self.compare_state.get("box_match_ok", False))

    def _slot_size_mm(self) -> float:
        if self.active_box is None:
            return 0.0
        if self.fill_dimension == "width":
            return float(self.active_box.get("width", 0.0))
        return float(self.active_box.get("length", 0.0))

    def _step_repeats_for_next_slot(self) -> int:
        slot_size = self._slot_size_mm()
        if slot_size <= 0.0:
            return 1
        return max(1, int(math.ceil(slot_size / self.y_step_default_mm)))

    def _base_full_after_next_place(self) -> bool:
        return (self.slot_offset_y_mm + self._slot_size_mm()) >= self.base_fill_length_mm

    def _reset_progress(self):
        self.placed_count = 0
        self.slot_offset_y_mm = 0.0
        self.wait_until = 0.0
        self.scan_deadline = 0.0

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    def loop(self):
        self._handle_screen_timeout()

        if self._safety_hold():
            return

        if self.state == "IDLE":
            if (
                self.auto_start
                and self._selected_box_ready()
                and self.selection_event_counter != self.handled_selection_event
            ):
                self.handled_selection_event = self.selection_event_counter
                self._capture_active_box()
                self._reset_progress()
                self._screen_on_optional()
                self.publish_debug(f"MISSION_START {self.active_box['name']}")
                self.set_state("INIT_AND_ZERO")
            return

        elif self.state == "INIT_AND_ZERO":
            if not self.state_once:
                ok = True
                ok &= self._send_macro("INIT_ALL")
                ok &= self._send_macro("ZERO_ALL")
                if not ok:
                    self.set_state("IDLE")
                    return
                self.publish_debug("INIT_ALL ZERO_ALL sent")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("SCAN_POSE")
            return

        elif self.state == "SCAN_POSE":
            if not self.state_once:
                ok = True
                ok &= self._send_macro("Z_UP")
                ok &= self._send_macro("DISK_CCW")
                ok &= self._send_macro("Y_FORWARD")
                if not ok:
                    self.set_state("IDLE")
                    return
                self.publish_debug("Scan pose sent")
                self._start_wait(self.command_wait_s)
                self.scan_deadline = self._now() + self.base_scan_timeout_s
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("WAIT_BASE")
            return

        elif self.state == "WAIT_BASE":
            if self._base_seen():
                self.publish_debug("BASE_SEEN")
                self.set_state("RETURN_FROM_SCAN")
                return

            if self._now() >= self.scan_deadline:
                self.publish_debug("BASE_SCAN_TIMEOUT")
                self.set_state("RETURN_FROM_SCAN")
            return

        elif self.state == "RETURN_FROM_SCAN":
            if not self.state_once:
                ok = True
                ok &= self._send_macro("Y_BACKWARD")
                ok &= self._send_macro("DISK_CW")
                if not ok:
                    self.set_state("IDLE")
                    return
                self.publish_debug("Returned from scan")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("WAIT_BOX_MATCH")
            return

        elif self.state == "WAIT_BOX_MATCH":
            if not self._selected_box_ready():
                self.publish_debug("Selection cleared")
                self.set_state("IDLE")
                return

            if self._box_match_ok():
                self.publish_debug("BOX_MATCH_OK")
                self.set_state("PLACE_DOWN")
            return

        elif self.state == "PLACE_DOWN":
            if not self.state_once:
                if not self._send_macro("Z_DOWN"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Place down sent")
                self._start_wait(self.place_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("PLACE_UP")
            return

        elif self.state == "PLACE_UP":
            if not self.state_once:
                if not self._send_macro("Z_UP"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Place up sent")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("TURN_LEFT_TO_END")
            return

        elif self.state == "TURN_LEFT_TO_END":
            if not self.state_once:
                if not self._send_macro("DISK_CCW"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Turned left")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("MOVE_LEFT_TO_END")
            return

        elif self.state == "MOVE_LEFT_TO_END":
            if not self.state_once:
                if not self._send_macro("X_BACKWARD"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Moved left to end")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("LOWER_AT_END")
            return

        elif self.state == "LOWER_AT_END":
            if not self.state_once:
                if not self._send_macro("Z_DOWN"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Lowered at end")
                self._start_wait(self.place_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("RAISE_AT_END")
            return

        elif self.state == "RAISE_AT_END":
            if not self.state_once:
                if not self._send_macro("Z_UP"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Raised at end")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("MOVE_BACK_FROM_END")
            return

        elif self.state == "MOVE_BACK_FROM_END":
            if not self.state_once:
                if not self._send_macro("X_FORWARD"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Moved back from end")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("TURN_RIGHT_TO_BOX")
            return

        elif self.state == "TURN_RIGHT_TO_BOX":
            if not self.state_once:
                if not self._send_macro("DISK_CW"):
                    self.set_state("IDLE")
                    return
                self.publish_debug("Turned right to box view")
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("ADVANCE_SLOT")
            return

        elif self.state == "ADVANCE_SLOT":
            if self._base_full_after_next_place():
                self.placed_count += 1
                self.publish_debug("BASE_FILLED")
                self.set_state("MISSION_DONE")
                return

            if not self.state_once:
                repeats = self._step_repeats_for_next_slot()
                ok = True
                for _ in range(repeats):
                    ok &= self._send_macro("Y_FORWARD")
                if not ok:
                    self.set_state("IDLE")
                    return

                self.slot_offset_y_mm += self._slot_size_mm()
                self.placed_count += 1
                self.publish_debug(
                    f"Advanced slot repeats={repeats} offset_y={self.slot_offset_y_mm:.2f}"
                )
                self._start_wait(self.command_wait_s)
                self.state_once = True
                return

            if self._wait_done():
                self.set_state("SCAN_POSE")
            return

        elif self.state == "MISSION_DONE":
            self.publish_state({"result": "done"})
            self.set_state("IDLE")
            return


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
