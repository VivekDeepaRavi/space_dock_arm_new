#!/bin/bash

sleep 1
/usr/bin/tmux -2 new-session -d -s brave
/usr/bin/tmux send-keys -t brave.0 "brave-browser --kiosk file:///home/ragav/Desktop/space_dock_arm/src/space_arm_ui-master/arm.html" ENTER   #change the browser and path according to your system.
