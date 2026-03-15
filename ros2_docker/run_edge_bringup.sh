#!/bin/bash
# ROS 2 Edge Bringup - Docker環境で起動

set -e

echo "=== Mobile Robot Edge Bringup ==="
echo ""
echo "Prerequisites:"
echo "1. edge_services must be running on host"
echo "   cd /home/pi/MobileRobot/edge_services"
echo "   python3 launch_all.py"
echo ""

# Check if edge_services is running
if ! netstat -tuln 2>/dev/null | grep -q ":8001"; then
    echo "WARNING: edge_services (port 8001) doesn't seem to be running!"
    echo "Please start edge_services first."
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Starting ROS 2 edge nodes in Docker..."
echo ""

cd /home/pi/MobileRobot/ros2_docker

sudo docker compose run --rm ros2 bash -c "\
    source /ros2_ws/install/setup.bash && \
    ros2 launch mobile_robot_edge ws_edge_bringup.launch.py"
