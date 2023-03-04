# Sensor Unit Tests

Details about the Dockerfile and Scripts
- Dockerfile installs ROS 2 Foxy and ROS 1 Noetic with ros1_bridge
- bash run_docker.sh: Mounts 2 workspaces, for ros2_ws and ros1_ws for ROS 2 foxy and ROS 1 noetic respectively. No workspace sourcing is set up by default
<br> <br>


# Running the code:

Requirements:
- Install Docker: https://docs.docker.com/engine/install/ubuntu/

## Docker Build:
```
sudo docker build -t lx_sensors .
```

## Run:
- Launch Docker Image
```
bash run_docker.sh
```

## General Commands:
- Launch ROS 1 Docker Terminal (Using set Alias)
``` 
sr1
```
- Launch ROS 2 Docker Terminal (Using set Alias)
``` 
sr2
```

## Running Sensors: 
Run inside a ROS 2 terminal
1. Source ROS 2 (using alias)
```
sr2
```
2. Building Code:
```
cd /home/ros2_ws
colcon build
```
3. Run sensors 
```
cd src 
bash run_sensors.sh
```

## Running Hardware Interface:
1. Build code:
    1. Launch Docker Image
    ```
    bash run_docker.sh
    ```
    2. Build ROS 2 code. In a new terminal
    ```
    bash terminal_docker.sh
    sr2 && colcon build
    ```
    3. Build ROS 1 code. In a new terminal
    ```
    bash terminal_docker.sh
    sr1 && catkin_make
    ```
    4. Build and upload Arduino code: Upload code in ros1_ws/src/arduino/arduino_rosserial.ino to the Arduino using Arduino IDE. Note that "#define USE_USBCON" should be added before importing <ros.h> in the arduino sketch. <br> <br>

2. Run rosserial node. In a new terminal
```
bash terminal_docker.sh
sr1 && roslaunch arduino_node arduino.launch
```

3. Run ROS 2 Hardware Mux. In a new terminal
```
bash terminal_docker.sh
sr2 && ros2 run hardware_mux hardware_mux_node
```

4. Run ROS 1 Bridge. This only bridges required topics specified in the yaml file betwen ROS 1 and ROS 2. In a new terminal:
```
bash terminal_docker.sh
sr1 && rosparam load /home/ros2_ws/bridge.yaml && sr2 && ros2 run ros1_bridge parameter_bridge
```



5. Emulate Rover Commands. In a new terminal
```
bash terminal_docker.sh
sr2 && ros2 topic pub /rover_hw_cmd lx_hw_msgs/msg/RoverCommand "{mobility_twist: {linear: {x: 0.5}, angular: {z: 0.2}}, actuator_speed: 1.0, drum_speed: 0.3}"
```



<hr>

## Error Logs:
- Failure to upload code to Arduino. Check the arduino port (something like /dev/ttyACMx). Replace x with the correct port number. Then run the following command:
```
sudo chmod 777 /dev/ttyACMx
```
- Changing owner of created folder in docker
```
sudo chown -R $USER:$USER <folder>/
```
- Vectornav package not built
    - change .hpp to .h in src/vn_sensor_msgs.cc
