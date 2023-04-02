### Base image
# AMD Architecture
FROM osrf/ros:humble-desktop
# ARM Architecture
# FROM arm64v8/ros:humble-ros-base

# Update & Upgrade
RUN sudo apt-get update && sudo apt-get upgrade -y


### Expose required ports
# TODO


### Install required applications
# VNC
# TODO

# Nano
RUN sudo apt-get install nano


### Copy source code
COPY ./src/lx_autonomy/src/ /home/lx_autonomy/lx_autonomy_ws/src/


### Customization
# Add utilities
COPY ./utilities/ /home/lx_autonomy/lx_autonomy_ws/utilities/


### Installation
# Run rosdep on src folder
RUN cd /home/lx_autonomy/lx_autonomy_ws && apt-get update && rosdep install -i --from-path src --rosdistro humble -y
# Link to allow sourcing
RUN rm /bin/sh && ln -s /bin/bash /bin/sh 
# Colcon build
RUN cd /home/lx_autonomy/lx_autonomy_ws && source /opt/ros/humble/setup.bash && colcon build
# Set work directory
WORKDIR /home/lx_autonomy/lx_autonomy_ws
RUN apt update && apt install ros-humble-joy-linux -y
# Add ascii script to bashrc (make sure to keep >> instead of > to avoid overwriting file)
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/utilities/lunarx_ascii.sh' >> ~/.bashrc