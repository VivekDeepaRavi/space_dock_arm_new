#!/bin/bash

/usr/bin/tmux -2 new-session -d -s moveit_custom4
/usr/bin/tmux send-keys -t moveit_custom4.0 "roslaunch moveit_custom4 demo.launch" ENTER

sleep 5
/usr/bin/tmux -2 new-session -d -s arm_controllers
/usr/bin/tmux send-keys -t arm_controllers.0 "roslaunch dynamixel_workbench_controllers one.launch" ENTER

sleep 5
/usr/bin/tmux -2 new-session -d -s node1
/usr/bin/tmux send-keys -t node1.0 "rosrun moveit_custom4 node1.py" ENTER

sleep 5
/usr/bin/tmux -2 new-session -d -s moveit_main
/usr/bin/tmux send-keys -t moveit_main.0 "rosrun moveit_custom4 move_grp_flow_finalurdf_ui_integrated2.py" ENTER

sleep 5
/usr/bin/tmux -2 new-session -d -s usb_cam_pack
/usr/bin/tmux send-keys -t usb_cam_pack.0 "rosrun usb_cam usb_cam_node _camera_name:='usb_cam' _camera_frame_id:='usb_cam'" ENTER

sleep 5
/usr/bin/tmux -2 new-session -d -s web
/usr/bin/tmux send-keys -t web.0 "roslaunch rosbridge_server rosbridge_websocket.launch" ENTER

sleep 5
/usr/bin/tmux -2 new-session -d -s brave
/usr/bin/tmux send-keys -t brave.0 "brave-browser --kiosk file:///home/ragav/Desktop/space_dock_arm/src/space_arm_ui-master/arm.html" ENTER   #change the browser and path according to your system.

# sleep 5
#Uncomment to launch camera in startup
# sleep 5
# usr/bin/tmux -2 new-session -d -s camera
# usr/bin/tmux send-keys -t camera.0 "rosrun moveit_custom4 555.py" ENTER
