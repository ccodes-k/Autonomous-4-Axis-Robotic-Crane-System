from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    moonraker_base_url = "http://192.168.1.210:7125"
    moonraker_api_key = ""

    zed_node = Node(
        package="project_4",
        executable="zed_node",
        name="zed_node",
        output="screen",
        parameters=[{
            "model_path": "/home/ros2/runs/box_detector11/weights/best.pt",
            "conf_thresh": 0.50,
        }],
    )

    moonraker_box_node = Node(
        package="project_4",
        executable="moonraker_box_node",
        name="moonraker_box_node",
        output="screen",
        parameters=[{
            "moonraker_base_url": moonraker_base_url,
            "moonraker_api_key": moonraker_api_key,
            "poll_interval": 0.5,
        }],
    )

    safety_camera_node = Node(
        package="project_4",
        executable="safety_camera_node",
        name="safety_camera_node",
        output="screen",
        parameters=[{
            "model_path": "/home/ros2/yolo11n.pt",
            "calib_path": "/home/ros2/c910_calibration.npz",
            "cam_index": 0,
            "conf_thresh": 0.45,
            "img_size": 640,
            "real_shoulder_width_m": 0.42,
            "min_bbox_width_px": 40,
            "smooth_window": 5,
            "stop_distance_m": 2.20,
            "estop_cooldown_s": 5.0,
            "require_hits": 2,
            "resume_poll_interval_s": 2.0,
            "moonraker_base_url": moonraker_base_url,
            "moonraker_api_key": moonraker_api_key,
        }],
    )

    box_compare_node = Node(
        package="project_4",
        executable="box_compare_node",
        name="box_compare_node",
        output="screen",
    )

    monitor_node = Node(
        package="project_4",
        executable="monitor_node",
        name="monitor_node",
        output="screen",
    )

    status_led_node = Node(
        package="project_4",
        executable="status_led_node",
        name="status_led_node",
        output="screen",
        parameters=[{
            "green_gpio": 4,
            "red_gpio": 22,
            "moonraker_base_url": moonraker_base_url,
            "moonraker_api_key": moonraker_api_key,
            "near_status_values": ["DANGER", "PAUSED_WAITING_RESUME"],
            "poll_interval": 0.5,
        }],
    )

    mission_controller_node = Node(
        package="project_4",
        executable="mission_controller_node",
        name="mission_controller_node",
        output="screen",
        parameters=[{
            "moonraker_base_url": moonraker_base_url,
            "moonraker_api_key": moonraker_api_key,
            "auto_start": True,
            "loop_period_s": 0.2,
            "screen_idle_timeout_s": 10.0,
            "command_wait_s": 1.0,
            "place_wait_s": 1.5,
            "base_scan_timeout_s": 5.0,
            "base_fill_length_mm": 600.0,
            "fill_dimension": "length",
            "y_step_default_mm": 20.0,
            "base_class_names": ["base"],
        }],
    )

    return LaunchDescription([
        zed_node,

        TimerAction(
            period=8.0,
            actions=[
                moonraker_box_node,
                safety_camera_node,
                box_compare_node,
                monitor_node,
                status_led_node,
            ],
        ),

        TimerAction(
            period=10.0,
            actions=[
                mission_controller_node,
            ],
        ),
    ])
