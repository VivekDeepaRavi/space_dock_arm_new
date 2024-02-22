cmake_minimum_required(VERSION 3.1.3)
project(moveit_custom4)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/move_grp_flow_finalurdf_ui_integrated.py
  scripts/move_grp_flow_finalurdf_ui_integrated2.py
  scripts/node1.py
  scripts/vivek.py
  scripts/aruco_posi_ros_1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
