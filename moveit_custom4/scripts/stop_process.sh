#!/bin/bash

# Closing Controllers and Other Nodes
/usr/bin/tmux send-keys -t arm_controllers.0 "" C-c
/usr/bin/tmux send-keys -t arm_controller.0 "" C-c
/usr/bin/tmux send-keys -t arduino_controllers "" C-c
/usr/bin/tmux send-keys -t moveit_custom4.0 "" C-c
/usr/bin/tmux send-keys -t moveit_main.0 "" C-c
/usr/bin/tmux send-keys -t node1.0 "" C-c
/usr/bin/tmux send-keys -t usb_cam_pack.0 "" C-c
/usr/bin/tmux send-keys -t web.0 "" C-c
/usr/bin/tmux send-keys -t brave.0 "" C-c

#/usr/bin/tmux send-keys -t camera.0 "" C-c
sleep 5

# Killing all tmux Servers
/usr/bin/tmux send-keys -t arm_controllers.0 "" C-c
/usr/bin/tmux send-keys -t arm_controller.0 "" C-c
/usr/bin/tmux send-keys -t arduino_controllers "" C-c
/usr/bin/tmux send-keys -t moveit_custom4.0 "" C-c
/usr/bin/tmux send-keys -t moveit_main.0 "" C-c
/usr/bin/tmux send-keys -t node1.0 "" C-c
/usr/bin/tmux send-keys -t usb_cam_pack.0 "" C-c
/usr/bin/tmux send-keys -t web.0 "" C-c
/usr/bin/tmux send-keys -t brave.0 "" C-c

#/usr/bin/tmux kill-session -t camera

