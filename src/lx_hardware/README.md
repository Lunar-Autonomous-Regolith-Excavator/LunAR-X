# LX Hardware Container

## Dockerfile
- Location: docker/lx_harware.Dockerfile
- Dockerfile installs an image with ROS 2 Foxy and ROS 1 Noetic with ros1_bridge
- No workspace sourcing is set up by default.
<br> <br>


## Running the code:
### Using the script and TMUX 
```
cd scripts
bash hardware_terminal.sh
bash run_hardware.sh
```

### Manual Launch
1. To launch the two docker containers (hardware and autonomy) 
    ```
    docker-compose up --build
    ```

    NOTE: 
    - To launch a terminal attached to the hardware container
        ```
        cd scripts
        bash hardware_terminal.sh
        ```

    - To source ROS 1 in the launched terminal (using set alias)
        ``` 
        sr1
        ```
    - To source ROS 2 in the launched terminal (using set alias)
        ```
        sr2
        ```

2. Build code:
    1. Build ROS 2 code. In a new terminal
        ```
        bash hardware_terminal.sh
        sr2 ; cd ros2_ws ; colcon build
        ```
    2. Build ROS 1 code. In a new terminal
        ```
        bash hardware_terminal.sh
        sr1 ; cd ros1_ws ; catkin_make
        ```
    3. Build and upload Arduino code: Upload code in ros1_ws/src/arduino/arduino_rosserial.ino to the Arduino using Arduino IDE. (Note that "#define USE_USBCON" has been added before importing <ros.h> in the arduino sketch to run rosserial with Arduino Due) <br> <br>

2. Run rosserial node. In a new terminal
    ```
    bash hardware_terminal.sh
    sr1 ; roslaunch arduino_node arduino.launch
    ```

3. Run ROS 2 Hardware Mux. In a new terminal
    ```
    bash hardware_terminal.sh
    sr2 ; ros2 run lx_hardware_mux hardware_mux_node
    ```

4. Run ROS 1 Bridge. This only bridges required topics specified in the yaml file betwen ROS 1 and ROS 2. In a new terminal:
    ```
    bash hardware_terminal.sh
    sr1 ; rosparam load /home/lx_hardware/ros2_ws/src/lx_packages/bridge.yaml ; sr2 ; ros2 run ros1_bridge parameter_bridge
    ```

5. If the autonomy docker is not up, we can emulate incoming rover commands. In a new terminal:
    ```
    bash hardware_terminal.sh
    sr2 ; ros2 topic pub /rover_hw_cmd lx_msgs/msg/RoverCommand "{mobility_twist: {linear: {x: 0.5}, angular: {z: 0.2}}, actuator_speed: 1.0, drum_speed: 0.3}"
    ```
6. Launch Husky Interface
    ```
    sr1; roslaunch husky_launch husky_launch.launch
    ```

<hr>
Calling the WeightEstimate Action (Hosted by the hardware_mux_node)
```
ros2 action send_goal /WeightEstimate lx_hardware_mux/action/WeightEstimate "{request: True}"
```

## Using TMUX
- See running sessions: ``` tmux ls ```
- Move around panes: ``` ctrl + b + arrow key ```
- Scroll: ``` ctrl + b + [ ```
- Kill: ``` ctrl + b + x ```
- Detach: ``` ctrl + b + d ```
- Kill Window after detaching: ``` tmux  kill-window -t <session_name>```
- Kill Window keyboard: ``` ctrl + b + & ```

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
    - change .hpp to .h in src/vn_sensor_msgs.cc (Added to dockerfile)

Husky
- The husky_base node connects to the husky, and the husky_control subscribes to cmd_vel and publishes to the husky_base node
- https://answers.ros.org/question/302019/roscore-and-sensors-node-are-not-starting-on-boot-clearpath-husky/
- Launch file at /etc/ros/noetic/ros.d/base.launch
- http://wiki.ros.org/Robots/Husky/hydro
- sudo service ros restart
- https://clearpathrobotics.com/assets/guides/foxy/husky/HuskyInstallSoftware.html Husky with ROS 2

- apt install and clone https://github.com/husky/husky_robot and sudo apt-get install ros-noetic-roslint ros-noetic-diagnostics