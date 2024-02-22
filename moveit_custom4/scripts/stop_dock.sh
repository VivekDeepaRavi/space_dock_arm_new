#!/bin/bash

/usr/bin/tmux send-keys -t moveit_main.0 "" C-c
sleep 5
/usr/bin/tmux kill-session -t moveit_main
