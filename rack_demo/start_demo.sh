#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
exec ros2 launch /home/ubuntu/rack_demo/rack_demo.launch.py
