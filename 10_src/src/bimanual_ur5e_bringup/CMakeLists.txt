cmake_minimum_required(VERSION 3.8)
project(bimanual_ur5e_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gello_bridge.py
  scripts/return_home.py
  scripts/set_servo_command_type.py
  scripts/twinnexus_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
