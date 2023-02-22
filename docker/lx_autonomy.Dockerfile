### Base image
# Using ros2-base configuration instead of desktop
FROM dustynv/ros:humble-ros-base-l4t-r35.2.1

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
COPY ../src/lx_autonomy/src/ /home/lx_autonomy/lx_autonomy_ws/src/

### Customization
# Add utilities
COPY ../utilities/ /home/lx_autonomy/lx_autonomy_ws/utilities/

# Add ascii script to bashrc (make sure to keep >> instead of > to avoid overwriting file)
RUN echo 'source /home/lx_autonomy/lx_autonomy_ws/utilities/lunarx_ascii.sh' >> ~/.bashrc