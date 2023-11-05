#!/bin/bash

# Automated running all autonomy nodes
# use bash command to make a file  ~/.tmux.conf and add lines to print pane name
# if [ ! -f ~/.tmux.conf ]; then
# add lines to tmux config to print pane name
# touch ~/.tmux.conf
# echo "set-option -g pane-border-status top" >> ~/.tmux.conf
# echo "set -g pane-border-format '[#[fg=white]#{?pane_active,#[bold],} #P #T #[fg=default,nobold]]'" >> ~/.tmux.conf

SESSION_NAME="autonomy"

if tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1; then
    # If session exists, attach to it
    tmux attach-session -t "$SESSION_NAME"
else
    # Launch tmux session
    tmux new-session -d -s autonomy

    # Split window into 2x2
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -h
    tmux select-pane -t 2 
    tmux split-window -h

    # Run commands in each pane
    tmux send-keys -t 0 "source /opt/ros/humble/setup.bash; cd /home/lx_autonomy/lx_autonomy_ws && rosdep install -i --from-path src --rosdistro humble -y && colcon build && source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash && ros2 launch lx_bringup_autonomy bringup_autonomy.launch.py" C-m
    sleep 20;
    tmux send-keys -t 1 "source /opt/ros/humble/setup.bash; colcon build --packages-up-to lx_localization && \
        source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash && ros2 launch lx_localization localization.launch.py" C-m
    
    # tmux send-keys -t 2 "source /opt/ros/humble/setup.bash; \
    #     source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m
    
    # tmux send-keys -t 3 "source /opt/ros/humble/setup.bash; colcon build --packages-select lx_perception && \
    #     source /home/lx_autonomy/lx_autonomy_ws/install/setup.bash && ros2 launch lx_perception perception.launch.py" C-m

    # Attach to tmux session
    tmux attach-session -t autonomy
fi