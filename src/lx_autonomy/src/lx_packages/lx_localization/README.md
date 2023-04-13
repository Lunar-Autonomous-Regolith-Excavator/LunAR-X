# Lunar-X Localization
Contains custom remap nodes and launch files to launch the robot_localization package for Lunar-X. robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see the documentation for robot_localization here: http://wiki.ros.org/robot_localization. 



## Running with a bag file
- The docker-compose mounts bags stored at /bags to /home/lx_autonomy/lx_autonomy_ws/bags in the autonomy container
- To run with a bag file, run the following command:
```
cd bags
ros2 bag play <file> --clock
```
This plays the file along with /clock messages, which are to simulate bag file clock (the param use_sim_time needs to be set true in the nodes to use the /clock time instead of the current time)
- Run the localization launch file (along with colcon build to update the package)
```
colcon build && ros2 launch lx_localization lx_ekf.launch.py 
```
This also loads up and launches a rviz2 instance to visualize the data.





