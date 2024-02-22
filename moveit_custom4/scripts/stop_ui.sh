#!/bin/bash

/usr/bin/tmux send-keys -t brave.0 "" C-c
sleep 1
/usr/bin/tmux kill-session -t brave
