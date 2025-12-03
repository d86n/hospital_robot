cmake_minimum_required(VERSION 3.0.2)
project(hospital_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  gazebo_ros
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  xacro
)

catkin_package(
  CATKIN_DEPENDS actionlib gazebo_ros geometry_msgs move_base_msgs nav_msgs rospy sensor_msgs std_msgs tf xacro
)

## Cài đặt các script Python (Node trung tâm của bạn)
# catkin_install_python(PROGRAMS
#   scripts/voice_control_node.py
#   scripts/navigation_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY
  launch
  param
  maps
  urdf
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)