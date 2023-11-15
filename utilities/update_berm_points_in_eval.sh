#!/bin/bash

# Call the berm evaluation service
ros2 service call berm_evaluation/requested_berm_points lx_msgs/srv/BermService "{
    berm: {
        berm_configuration: [
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 5.0,
            "y": 3.81,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 5.0,
            "y": 1.28,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 5.0,
            "y": 6.0,
            "z": 0.0
        }
    }
]

    }
}"
