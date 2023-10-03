### Base image with ros bridge
FROM ros:foxy-ros1-bridge-focal


### Install ROS & relevant drivers 
# Install Rosserial
RUN apt update && apt install ros-noetic-rosserial-arduino ros-noetic-rosserial -y

# Install Realsense Drivers
RUN apt-get install -y ros-foxy-realsense2-camera

# Install tool height Packages
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install transforms3d

# Install ROS utility Packages
RUN apt install ros-noetic-rqt ros-noetic-rqt-graph -y
RUN apt install ros-foxy-rviz2 ros-foxy-tf-transformations -y


# Link to allow sourcing
RUN rm /bin/sh && ln -s /bin/bash /bin/sh 

# Clone Realsense and Vectornav Repos, and install dependencies
RUN mkdir -p /home/lx_hardware/ros2_ws/src/sensor_packages && cd /home/lx_hardware/ros2_ws/src/sensor_packages && \
    git clone --single-branch --branch ros2 https://github.com/dawonn/vectornav.git
    # git clone --single-branch --branch ros2-development https://github.com/IntelRealSense/realsense-ros.git
# Replace .hpp with .h in vectornav package to build with ros2 foxy
RUN sed -i 's/tf2_geometry_msgs.hpp/tf2_geometry_msgs.h/g' \
    /home/lx_hardware/ros2_ws/src/sensor_packages/vectornav/vectornav/src/vn_sensor_msgs.cc 
# Rosdep
RUN apt-get update && cd /home/lx_hardware/ros2_ws && rosdep install --from-paths src --ignore-src -r -y --rosdistro=foxy
# Colcon build
RUN cd /home/lx_hardware/ros2_ws/ && source /opt/ros/foxy/setup.bash && colcon build

# Alias for sourcing
# sr1 -> source ros 1
# sr2 -> source ros 2
RUN echo "alias sr1='source /opt/ros/noetic/setup.bash; source /home/lx_hardware/ros1_ws/devel/setup.bash'" >> ~/.bashrc
RUN echo "alias sr2='source /opt/ros/foxy/setup.bash; source /home/lx_hardware/ros2_ws/install/setup.bash'" >> ~/.bashrc

# Change work directory
WORKDIR /home/lx_hardware/

# Install Husky Packages
RUN apt-get update && apt install ros-noetic-husky* -y
RUN mkdir -p /home/lx_hardware/ros1_ws/src && cd /home/lx_hardware/ros1_ws/src && \
     git clone --single-branch --branch noetic-devel https://github.com/husky/husky_robot.git
RUN apt-get update && cd /home/lx_hardware/ros1_ws && source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y --rosdistro=noetic
RUN apt update && apt install ros-noetic-roslint ros-noetic-diagnostics -y
RUN cd /home/lx_hardware/ros1_ws && source /opt/ros/noetic/setup.bash && catkin_make

# Install Utility Packages
RUN apt-get update && \
    apt-get install -y nano tmux
    


# Mount ROS2 workspace and colcon build
COPY ./src/lx_hardware/ros2_ws/src/lx_packages/ /home/lx_hardware/ros2_ws/src/lx_packages/
RUN cd /home/lx_hardware/ros2_ws/ && source /opt/ros/foxy/setup.bash && colcon build

# Enter with bash
CMD ["bash"]
