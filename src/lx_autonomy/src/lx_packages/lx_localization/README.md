# Lunar-X Localization
Contains custom remap nodes and launch files to launch the robot_localization package for Lunar-X. robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see the documentation for robot_localization here: http://wiki.ros.org/robot_localization. 

## Brief Overview
- Two EKF nodes run
    - The first node fuses IMU and Wheel Encoder data, and publishes `/odometry/ekf_odom_node` in the odom frame
    - The second node fuses IMU, Wheel Encoder, and Total Station Data, and publishes `/odometry/ekf_map_node` in the map frame
- A remap_msgs_localization node is run for the following reasons
    - To remap `/husky_velocity_controller/odom` msg to `/husky_odom`, changing its frame to `base_link` and adding the required covariance values
    - The incoming message from the total station `/total_station_prism` is for (map->total_station_prism). This node transforms the message to (map->base_link) by using the static transform (base_link->total_station_prism) and publishes it as `/total_station_pose_map`

## Running with a bag file
- The docker-compose mounts bags stored at /bags to /home/lx_autonomy/lx_autonomy_ws/bags in the autonomy container
- To run with a bag file, run the following command:
```
cd bags
ros2 bag play <file> --clock
```
This plays the file along with /clock messages, which are to simulate bag file clock (the param use_sim_time is to be set true in the nodes to use the /clock time instead of the current time)
- Run the localization launch file (along with colcon build to account for any changes to the code)
```
colcon build && ros2 launch lx_localization lx_ekf.launch.py use_sim_time:=True
```
This also loads up and launches a rviz2 instance to visualize the data.





