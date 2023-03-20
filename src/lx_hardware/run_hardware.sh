#!/bin/bash

SESSION_NAME="hardware"

if tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1; then
    # if session exists, attach to it
    tmux attach-session -t "$SESSION_NAME"
else
    # Launch tmux session
    tmux new-session -d -s hardware

    # Split window into 2x2 grid
    tmux split-window -h
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -v

    # Run commands in each pane
    tmux send-keys -t 0 "sr1; roslaunch arduino_node arduino.launch" C-m
    tmux send-keys -t 1 "sr1; roslaunch husky_launch husky_launch.launch" C-m
    tmux send-keys -t 2 "sr1; rosparam load /home/lx_hardware/ros2_ws/src/lx_packages/bridge.yaml; sr2; ros2 run ros1_bridge parameter_bridge" C-m
    tmux send-keys -t 3 "sr2; cd /home/lx_hardware/ros2_ws && colcon build && ros2 run hardware_mux hardware_mux_node" C-m
    # Attach to tmux session
    tmux attach-session -t hardware
fi