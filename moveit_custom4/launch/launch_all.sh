#!/bin/bash

#starting controllers and bringup
sleep 1
/usr/bin/tmux -2 new-session -d -s arduino
/usr/bin/tmux send-keys -t arduino.0 "roslaunch rosserial_python serial_node.py port:=/dev/ttyACM1" ENTER
sleep 2

sleep 1
/usr/bin/tmux -2 new-session -d -s moveit
/usr/bin/tmux send-keys -t moveit.0 "moveit_custom4 demo.launch" ENTER
sleep 8

/usr/bin/tmux -2 new-session -d -s arm
/usr/bin/tmux send-keys -t arm.0 "roslaunch dynamixel_workbench_controllers one.launch" ENTER
sleep 5

/usr/bin/tmux -2 new-session -d -s end_eff
/usr/bin/tmux send-keys -t end_eff.0 "roslaunch dynamixel_workbench_controllers end_eff.launch" ENTER
sleep 5

