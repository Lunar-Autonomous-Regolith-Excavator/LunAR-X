utilities/start_pc_handling.sh
ros2 service call lx_mapping/pc_handler lx_msgs/srv/Map 'start: true' && ros2 bag play ../mapping_13-10/rosbag2_2023_10_13-22_03_10/ -r 5
