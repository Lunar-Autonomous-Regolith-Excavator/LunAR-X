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
            "x": 5.91,
            "y": 4.29,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 5.98,
            "y": 3.9,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 6.00,
            "y": 3.5,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 5.98,
            "y": 3.1,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 5.91,
            "y": 2.71,
            "z": 0.0
        }
    }
]

    }
}"
