# ROS2 Humble + PyTorch + Gazebo with GPU support
FROM osrf/ros:humble-desktop

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch with CUDA support
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Install other Python dependencies
RUN pip3 install --no-cache-dir \
    ultralytics \
    opencv-python \
    timm

# Pin numpy<2 AFTER other packages (ultralytics pulls numpy>=2, but cv_bridge needs <2)
RUN pip3 install --no-cache-dir "numpy<2"

# Set up workspace
WORKDIR /ros_ws

# Source ROS2 automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi" >> ~/.bashrc

# Default command
CMD ["bash"]
