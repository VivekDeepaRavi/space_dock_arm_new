#!/bin/bash

/usr/bin/tmux -2 new-session -d -s target_check
/usr/bin/tmux send-keys -t target_check.0 "rosrun moveit_custom4 aruco_posi_ros_1.py" ENTER

sleep 5
#code for camera