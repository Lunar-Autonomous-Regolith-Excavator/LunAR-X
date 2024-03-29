### Base image
# AMD Architecture
# FROM osrf/ros:humble-desktop
# ARM Architecture
FROM ros:humble-ros-base

# Update & Upgrade
RUN sudo apt-get update && sudo apt-get upgrade -y


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

### Copy source code
COPY ./src/lx_autonomy/src/ /home/lx_autonomy/lx_autonomy_ws/src/

### Customization
# Add utilities
COPY ./utilities/ /home/lx_autonomy/lx_autonomy_ws/utilities/

# Run rosdep on src folder
RUN cd /home/lx_autonomy/lx_autonomy_ws && apt-get update && rosdep install -i --from-path src --rosdistro humble -y

# Colcon build
RUN cd /home/lx_autonomy/lx_autonomy_ws && source /opt/ros/humble/setup.bash && colcon build

# Set work directory
WORKDIR /home/lx_autonomy/lx_autonomy_ws

# Auto source ROS 2 workspace
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash' >> ~/.bashrc

# Add ascii script to bashrc (make sure to keep >> instead of > to avoid overwriting file)
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/utilities/lunarx_ascii.sh' >> ~/.bashrc