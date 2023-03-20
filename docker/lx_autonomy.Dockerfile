### Base image
# Using ros2-base configuration instead of desktop
# FROM dustynv/ros:humble-ros-base-l4t-r35.2.1
FROM osrf/ros:humble-desktop
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

# run rosdep on src folder
RUN cd /home/lx_autonomy/lx_autonomy_ws && apt-get update && rosdep install -i --from-path src --rosdistro humble -y
# Allow sourcing
RUN rm /bin/sh && ln -s /bin/bash /bin/sh 
RUN cd /home/lx_autonomy/lx_autonomy_ws && source /opt/ros/humble/setup.bash && colcon build

# Set workdir
WORKDIR /home/lx_autonomy/lx_autonomy_ws

# Add ascii script to bashrc (make sure to keep >> instead of > to avoid overwriting file)
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/utilities/lunarx_ascii.sh' >> ~/.bashrc