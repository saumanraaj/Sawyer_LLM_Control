cmake_minimum_required(VERSION 3.0.2)
project(sawyer_vision_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  moveit_commander
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vision_node.py
  scripts/pickup_node.py
  scripts/joint_states_relay.py
  scripts/zed_raw_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
