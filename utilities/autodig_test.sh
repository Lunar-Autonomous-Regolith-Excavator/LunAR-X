# ros2 service call /param_server_node/set_parameters rcl_interfaces/srv/SetParameters\
#     "{parameters: [{name: \"autodig.pid_outer\", value: {type: 8, double_array_value: [1, 0.1, 1.5]}},\
#     {name: \"autodig.pid_inner\", value: {type: 8, double_array_value: [1, 0.1, 1.5]}}]}"

ros2 action send_goal /operations/autodig_action lx_msgs/action/AutoDig "{}"