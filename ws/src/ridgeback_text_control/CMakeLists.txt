cmake_minimum_required(VERSION 3.0.2)
project(ridgeback_text_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ewellix_scu_rs232.py
  scripts/llm_command_bridge.py
  scripts/map_visualizer.py
  scripts/text_command_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
