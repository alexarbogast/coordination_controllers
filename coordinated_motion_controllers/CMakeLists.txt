cmake_minimum_required(VERSION 3.5)
project(coordinated_motion_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(coordinated_control_msgs REQUIRED)
find_package(taskspace_control_msgs REQUIRED)
find_package(taskspace_controllers REQUIRED)
find_package(task_priority_controllers REQUIRED)
find_package(axially_symmetric_controllers REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)

## positioner objective parameters
generate_parameter_library(
  match_configuration_parameters
  src/positioner_objectives/match_configuration_parameters.yaml
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

## positioner_objectives
add_library(positioner_objectives SHARED
  src/positioner_objectives/positioner_objective_plugin.cpp
  src/positioner_objectives/minimize_velocity.cpp
  src/positioner_objectives/match_configuration.cpp
  # src/positioner_objectives/taskspace_anchor.cpp
)
target_link_libraries(positioner_objectives
  match_configuration_parameters
)
ament_target_dependencies(positioner_objectives
  rclcpp
  urdf
  kdl_parser
  sensor_msgs
  coordinated_control_msgs
  taskspace_control_msgs
  taskspace_controllers
  task_priority_controllers
  axially_symmetric_controllers
  hardware_interface
  controller_interface
  realtime_tools
  pluginlib
)

pluginlib_export_plugin_description_file(
  coordinated_motion_controllers
  positioner_objective_plugins.xml
)

## coordinated_motion_controllers
add_library(${PROJECT_NAME} SHARED
  src/coordinated_controller_base.cpp
  src/pose_controller.cpp
  src/axially_symmetric_controller.cpp
  src/positioner_controller.cpp
  src/positioner_objectives/positioner_objective_plugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  urdf
  kdl_parser
  sensor_msgs
  coordinated_control_msgs
  taskspace_control_msgs
  taskspace_controllers
  task_priority_controllers
  axially_symmetric_controllers
  hardware_interface
  controller_interface
  realtime_tools
  pluginlib
)

# Export controller plugins
pluginlib_export_plugin_description_file(
  controller_interface
  coordinated_controller_plugins.xml
)

## Installation
install(
  TARGETS
    ${PROJECT_NAME}
    positioner_objectives
    match_configuration_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES
    coordinated_controller_plugins.xml
    positioner_objective_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export interfaces for downstream packages
ament_export_include_directories(include)
ament_export_libraries(
   ${PROJECT_NAME}
  positioner_objectives
)
ament_export_dependencies(
  rclcpp
  urdf
  kdl_parser
  sensor_msgs
  coordinated_control_msgs
  taskspace_control_msgs
  taskspace_controllers
  task_priority_controllers
  axially_symmetric_controllers
  hardware_interface
  controller_interface
  realtime_tools
  pluginlib
  Eigen3
)

ament_package()
