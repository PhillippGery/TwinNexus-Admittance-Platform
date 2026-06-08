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
  scripts/admittance_keyboard_teleop.py
  scripts/moveit_servo_to_admittance.py
  scripts/set_servo_command_type.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
