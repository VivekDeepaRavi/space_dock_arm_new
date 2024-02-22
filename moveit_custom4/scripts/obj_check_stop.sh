#!/bin/bash

/usr/bin/tmux send-keys -t target_check.0 "" C-c
sleep 5
/usr/bin/tmux kill-session -t target_check
