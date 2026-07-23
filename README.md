# Autonomous-4-Axis-Robotic-Crane-System


## Overview

A third-party logistics workflow needed a safer and more repeatable way to perform side pallet-loading while accounting for changing box sizes, cage occupancy, and available loading space. 


### Idea
 - Designed the ROS 2 software architecture for an autonomous 4-axis robotic crane workflow.
 - Trained and deployed YOLOv11 on an NVIDIA Jetson Orin Nano to detect box sizes, estimate cage occupancy, and identify available loading space.
 - Built ROS 2 nodes to translate real-time vision outputs into G-code commands for robotic-controller motion execution.
 - Integrated perception, AI inference, motion planning, and robotic control into an end-to-end autonomous pick-and-place pipeline.
