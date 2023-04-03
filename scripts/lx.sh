# #!/bin/bash
# # write a script to launch 2 terminator instances and run 2 commands in each
# # launch terminator in the same directory as the script
cd "$(dirname "$0")"
xhost +local:docker &&

# Get the screen resolution
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | cut -d 'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | cut -d 'x' -f2)

# Calculate the width and height of each window
WINDOW_WIDTH=$(((SCREEN_WIDTH - 80)/ 2))
WINDOW_OFFSET_2=$((WINDOW_WIDTH + 75))
WINDOW_HEIGHT=$((SCREEN_HEIGHT - 40)) # subtract 40 to account for the top panel

# Open the left window
terminator --command="bash -c 'docker exec -it lx_autonomy bash -c \"bash init_autonomy.sh\"; exec bash'" --geometry=${WINDOW_WIDTH}x${WINDOW_HEIGHT}+0+0  &

# Open the right window
terminator --geometry=${WINDOW_WIDTH}x${WINDOW_HEIGHT}+${WINDOW_OFFSET_2}+0 --command="bash -c 'docker exec -it lx_hardware bash -c \"bash init_hardware.sh\"; exec bash'"