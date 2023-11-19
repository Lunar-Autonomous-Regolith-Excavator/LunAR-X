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
            "x": 6.0,
            "y": 2.4,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 6.0,
            "y": 2.8,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 6.0,
            "y": 3.2,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 6.0,
            "y": 3.6,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 6.0,
            "y": 4.0,
            "z": 0.0
        }
    }
]

    }
}"
