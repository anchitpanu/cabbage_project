#!/bin/bash

echo "Starting PC system..."

# ROS settings
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS Jazzy
source /opt/ros/jazzy/setup.bash
source ~/plant/cabbage_project/ros2_ws/install/setup.bash

echo "ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

# Start Dashboard
gnome-terminal --title="AGRIBOT DASHBOARD" -- bash -c "
cd ~/plant/cabbage_project/agribot-dashboard
npm run dev
exec bash
"

# Start Vision Nodes
gnome-terminal --title="VISION NODES" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/plant/cabbage_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch robot_vision vision_system.launch.py
exec bash
"

echo "All PC systems launched"
