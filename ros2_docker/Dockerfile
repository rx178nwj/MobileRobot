FROM ros:jazzy-ros-base

# Install additional packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-v4l2-camera \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-camera-calibration-parsers \
    ros-jazzy-camera-info-manager \
    ros-jazzy-compressed-image-transport \
    i2c-tools \
    v4l-utils \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (skip numpy and opencv-python as they're from ROS packages)
RUN pip3 install --break-system-packages smbus2 websockets

# Setup workspace
WORKDIR /ros2_ws

# Source ROS 2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
