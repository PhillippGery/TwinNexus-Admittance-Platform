cmake_minimum_required(VERSION 3.8)
project(ur5e_admittance_control)

# 1. Find dependencies first
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

# 2. Define the library
add_library(admittance_controller SHARED src/admittance_controller.cpp)

# 3. Include directories
target_include_directories(admittance_controller PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. Link dependencies
ament_target_dependencies(admittance_controller
  controller_interface
  pluginlib
  hardware_interface
)

# 5. Export the plugin
pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)

# 6. Installation
install(TARGETS admittance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()