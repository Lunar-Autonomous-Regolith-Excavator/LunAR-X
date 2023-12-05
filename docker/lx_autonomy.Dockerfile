### Base image
# AMD Architecture
# FROM osrf/ros:humble-desktop
# ARM Architecture
FROM ros:humble-ros-base

# Update & Upgrade
RUN sudo apt-get update && sudo apt-get upgrade -y

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

### Expose required ports
# TODO


### Install required applications
# VNC
# TODO

# Nano tmux
RUN apt update && sudo apt-get install nano tmux -y

### Clone github packages
# Clone robot_localization package
# RUN mkdir -p /home/lx_autonomy/lx_autonomy_ws/src/git_packages && cd /home/lx_autonomy/lx_autonomy_ws/src/git_packages && \
#         git clone --single-branch --branch ros2 https://github.com/cra-ros-pkg/robot_localization.git
RUN apt update && apt install ros-humble-robot-localization -y

### Installation
# Link to allow sourcing
RUN rm /bin/sh && ln -s /bin/bash /bin/sh 
# install rviz2
RUN apt update && apt install ros-humble-rviz2* -y
# install rqt
RUN apt update && apt install ros-humble-rqt* -y
# install pip3
RUN apt update && apt install python3-pip -y
# install foxglove bridge
RUN apt-get install ros-humble-foxglove-bridge -y
# install joy
RUN apt update && apt install ros-humble-joy-linux -y
# hybrid a* dependencies
RUN apt update && apt-get install libompl-dev libeigen3-dev libceres-dev libopencv-dev -y
# Google OR-Tools
RUN apt update && apt install -y build-essential cmake lsb-release wget
WORKDIR /home/
RUN wget https://github.com/google/or-tools/releases/download/v9.8/or-tools_amd64_ubuntu-22.04_cpp_v9.8.3296.tar.gz
RUN tar -xf or-tools_amd64_ubuntu-22.04_cpp_v9.8.3296.tar.gz
RUN mv or-tools_x86_64_Ubuntu-22.04_cpp_v9.8.3296 or-tools
WORKDIR /home/or-tools
RUN make test

# Install ffmpeg to save video
RUN apt update && apt install ffmpeg -y

### Copy source code
COPY ./src/lx_autonomy/src/ /home/lx_autonomy/lx_autonomy_ws/src/

### Customization
# Add utilities
COPY ./utilities/ /home/lx_autonomy/lx_autonomy_ws/utilities/

# Run rosdep on src folder
RUN cd /home/lx_autonomy/lx_autonomy_ws && DEBIAN_FRONTEND=noninteractive rosdep update && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src -r -y

# Colcon build
RUN cd /home/lx_autonomy/lx_autonomy_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 4

# Set work directory
WORKDIR /home/lx_autonomy/lx_autonomy_ws

# Auto source ROS 2 workspace
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash' >> ~/.bashrc

# Add ascii script to bashrc (make sure to keep >> instead of > to avoid overwriting file)
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/utilities/lunarx_ascii.sh' >> ~/.bashrc