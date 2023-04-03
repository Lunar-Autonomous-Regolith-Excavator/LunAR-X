#!/bin/bash

# Automated running all hardware nodes
# use bash command to make a file  ~/.tmux.conf and add lines to print pane name
# if [ ! -f ~/.tmux.conf ]; then
# add lines to tmux config to print pane name
# touch ~/.tmux.conf
# echo "set-option -g pane-border-status top" >> ~/.tmux.conf
# echo "set -g pane-border-format '[#[fg=white]#{?pane_active,#[bold],} #P #T #[fg=default,nobold]]'" >> ~/.tmux.conf

SESSION_NAME="hardware"

if tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1; then
    # If session exists, attach to it
    tmux attach-session -t "$SESSION_NAME"
else
    # Launch tmux session
    tmux new-session -d -s hardware

    # Split window into 3x2 grid
    tmux split-window -v
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -h
    tmux select-pane -t 2
    tmux split-window -h
    tmux select-pane -t 4
    tmux split-window -h

    # Run commands in each pane (add sleeps to wait for roscore to start)
    tmux send-keys -t 0 "sr1; sleep 2; roslaunch arduino_node arduino.launch" C-m
    # tmux send-keys -t 1 "sr1; sleep 2; roslaunch husky_launch husky_launch.launch" C-m
    tmux send-keys -t 2 "sr1; sleep 2; rosparam load /home/lx_hardware/ros2_ws/src/lx_packages/bridge.yaml; sr2; ros2 run ros1_bridge parameter_bridge" C-m
    # tmux send-keys -t 3 "sr2; cd /home/lx_hardware/ros2_ws && colcon build && sr2 && ros2 run lx_hardware_mux hardware_mux_node" C-m
    tmux send-keys -t 3 "sr2; cd /home/lx_hardware/ros2_ws && colcon build && sr2 && ros2 launch lx_bringup_hardware bringup_hardware" C-m
    tmux send-keys -t 4 "sr1; roscore" C-m
    tmux send-keys -t 5 "bash" C-m

    # Attach to tmux session
    tmux attach-session -t hardware
fi