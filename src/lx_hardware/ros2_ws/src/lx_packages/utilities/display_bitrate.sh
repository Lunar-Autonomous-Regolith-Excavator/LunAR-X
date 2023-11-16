#!/bin/bash

# Get list of all active ROS2 topics
topics=$(ros2 topic list)

echo "Estimating bitrate for ROS2 topics:"

# Loop through each topic and estimate bitrate
for topic in $topics
do
    echo "Topic: $topic"

    # Get publishing frequency using ros2 topic hz
    bitrate=$(ros2 topic bw $topic | grep -oP "(?<=average:\s)[0-9.]+" | head -n 1)

    if [ ! -z "$bitrate" ]; then
        echo "  Publishing bitrate: $bitrate MBps"

        # You can convert bytes to bits if required (1 byte = 8 bits)
        # bitrate_bits=$(echo "$bitrate * 8" | bc)
        # echo "  Estimated Bitrate: $bitrate_bits bits/s"
    else
        echo "  No messages received yet."
    fi

    echo ""
done
