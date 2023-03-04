FROM ros:foxy-ros1-bridge-focal

# Install Rosserial
RUN apt update && apt install ros-noetic-rosserial-arduino ros-noetic-rosserial -y

# Install Realsense Drivers
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    rm -rf /var/lib/apt/lists/*
RUN add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Clone Realsense and Vectornav Repos, and install dependencies
RUN mkdir -p /home/ros2_ws/src/sensor_packages && cd /home/ros2_ws/src/sensor_packages && \
    git clone --single-branch --branch ros2 https://github.com/dawonn/vectornav.git && \
    git clone --single-branch --branch ros2-development https://github.com/IntelRealSense/realsense-ros.git
RUN apt-get update && cd /home/ros2_ws && rosdep install --from-paths src --ignore-src -r -y

RUN echo "alias sr1='source /opt/ros/noetic/setup.bash; source /home/ros1_ws/devel/setup.bash'" >> ~/.bashrc
RUN echo "alias sr2='source /opt/ros/foxy/setup.bash; source /home/sensor_test_ws/install/setup.bash'" >> ~/.bashrc

# Install Utility Packages
RUN apt-get update && \
    apt-get install -y nano

CMD ["bash"]