#!/bin/bash

# Sample points for berm evaluation
berm_eval_points=(
    '{header: {frame_id: "map"}, point: {x: 1.0, y: 2.0, z: 0.0}}',
    '{header: {frame_id: "map"}, point: {x: 3.0, y: 4.0, z: 0.0}}',
    '{header: {frame_id: "map"}, point: {x: 5.0, y: 6.0, z: 0.0}}'
)

# Call the berm evaluation service
ros2 service call berm_evaluation/requested_berm_points lx_msgs/srv/BermService "{
    berm: {
        berm_configuration: [
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0
        }
    },
    {
        "header": {
            "frame_id": "map"
        },
        "point": {
            "x": 3.0,
            "y": 4.0,
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
