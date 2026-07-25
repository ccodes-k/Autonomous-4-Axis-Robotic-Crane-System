from setuptools import find_packages, setup

package_name = "project_4"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/monitor.launch.py",
            "launch/full_system.launch.py",   # add this if you create it
        ]),
        ("share/" + package_name + "/config", [
            "config/boxes.yaml",
            "config/workspace.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2",
    maintainer_email="ros2@todo.todo",
    description="Project 4 ROS2 camera and Moonraker monitor",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zed_node = project_4.zed_node:main",
            "safety_camera_node = project_4.safety_camera_node:main",
            "moonraker_box_node = project_4.moonraker_box_node:main",
            "box_compare_node = project_4.box_compare_node:main",
            "monitor_node = project_4.monitor_node:main",
            "status_led_node = project_4.status_led_node:main",
            "mission_controller_node = project_4.mission_controller_node:main",
        ],
    },
)
