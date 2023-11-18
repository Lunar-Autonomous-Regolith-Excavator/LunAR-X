#!/bin/bash

ros2 service call /berm_evaluation/berm_progress lx_msgs/srv/BermProgressEval "{ need_metrics: true }"

