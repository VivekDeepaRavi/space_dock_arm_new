#!/bin/bash

/usr/bin/tmux send-keys -t moveit_main.0 "" C-c
sleep 2
/usr/bin/tmux kill-session -t moveit_main

sleep 5

#!/bin/bash

/usr/bin/tmux -2 new-session -d -s moveit_main
/usr/bin/tmux send-keys -t moveit_main.0 "rosrun moveit_custom4 move_grp_flow_finalurdf_ui_integrated.py" ENTER

sleep 5
#code for camera