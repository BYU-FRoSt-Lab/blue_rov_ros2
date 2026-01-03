ARG ROS_DISTRO=humble 
FROM ros:${ROS_DISTRO}

# System dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-sbg-driver \
    ros-${ROS_DISTRO}-ouster-ros \
    ros-${ROS_DISTRO}-ntrip-client \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*


# Additional optional helpful debug tools
RUN apt-get update && apt-get install -y \
    netcat \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /root/ros2_ws/src

# --- Source ROS2 ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
    
WORKDIR /root/ros2_ws
