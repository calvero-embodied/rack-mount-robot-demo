#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
exec python3 /home/ubuntu/rack_demo/rack_session.py "$@"
