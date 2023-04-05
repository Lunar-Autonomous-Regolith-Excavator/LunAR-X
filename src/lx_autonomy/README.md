# LX Autonomy Container

## Dockerfile
- Location: docker/lx_autonomy.Dockerfile
- Dockerfile installs an image with ROS 2 Humble
<br> <br>


## Running the code:
### Using the script and TMUX 
```
cd scripts
bash autonomy_terminal.sh
bash init_autonomy.sh
```

### Manual Launch
1. Start and enter the container with `docker-compose up --build`
2. Enter docker container using `docker exec -it <container_name> bash`
3. [ All the subsequent instructions are for inside the container ]
4. Change directory to `/home/lx_autonomy/lx_autonomy_ws/`
5. Run `colcon build`
6. Make sure `/joy` topic is being published (joystick is connected with `joy_linux` node running)
7. Run `. install/setup.bash`
8. Run `ros2 launch lx_bringup_autonomy bringup_autonomy.launch.py`
<br> <br>


# Description

##  External Interface
### Subscriptions
- /joy <br>
sensor_msgs::msg::Joy <br>
Joystick raw input

### Publishing
- /rover_teleop_cmd <br>
lx_msgs::msg::RoverCommand <br>
Teleop command passthrough to command_mux_node

### Service client
- /param_server_node/set_parameters <br>
rcl_interfaces::srv::SetParameters <br>
Client to set or change global params on param_server_node

### Summary
- Based on user inputs, guides the robot operation
- Joystick buttons set the lock, operation mode and task mode
- If the op mode is set to teleop, will passthrough the joystick teleop commands to the command mux via RoverCommand data type
- If no joystick data received for 3 seconds, will set the rover to lock for safety.
- TODO : Add berm inputs
- TODO : Add initial getting of parameters

<hr>

## Param Server
### Summary
- Provides access to globally crucial variables like current lock status, operation & task modes, actuation limits etc. 

<hr>

## Command Mux
### Subscriptions
- /rover_teleop_cmd <br>
lx_msgs::msg::RoverCommand
Teleop command from joystick via external_interface_node

- /rover_auto_cmd <br>
lx_msgs::msg::RoverCommand
Autonomy command

### Publishing
- /rover_hw_cmd <br>
lx_msgs::msg::RoverCommand
Command published to the lx_hardware container's hardware_mux_node

### Services
### Summary
- Switches between teleop and autonomous commands based on the operation mode
- Enforces actuation limits
- TODO : Test autonomous input handling
- TODO : Add initial getting of parameters
<hr>

## LX Library
TODO