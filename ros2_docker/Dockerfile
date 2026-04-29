FROM ros:jazzy-ros-base

# Install additional packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    swig \
    ros-jazzy-cv-bridge \
    ros-jazzy-v4l2-camera \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-camera-calibration-parsers \
    ros-jazzy-camera-info-manager \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-navfn-planner \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-collision-monitor \
    i2c-tools \
    v4l-utils \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install YDLidar SDK (C library + Python bindings)
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/YDLidar-SDK && \
    cmake -S /tmp/YDLidar-SDK -B /tmp/YDLidar-SDK/build -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/YDLidar-SDK/build -j4 && \
    cmake --install /tmp/YDLidar-SDK/build && \
    pip3 install --break-system-packages /tmp/YDLidar-SDK && \
    rm -rf /tmp/YDLidar-SDK

# Install Python dependencies
RUN pip3 install --break-system-packages smbus2 websockets openai pyserial numpy

# Setup workspace
WORKDIR /ros2_ws

# Source ROS 2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
