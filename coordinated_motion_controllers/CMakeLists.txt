cmake_minimum_required(VERSION 3.0.2)
project(coordinated_motion_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  dynamic_reconfigure
  pluginlib
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TaskspaceAnchor.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} positioner_objectives
  CATKIN_DEPENDS
    roscpp
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
    dynamic_reconfigure
    pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## positioner_objectives
add_library(positioner_objectives
  src/positioner_objectives/positioner_objective_plugin.cpp
  src/positioner_objectives/minimize_velocity.cpp
  src/positioner_objectives/match_configuration.cpp
  src/positioner_objectives/taskspace_anchor.cpp
)
target_link_libraries(positioner_objectives ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(positioner_objectives ${catkin_EXPORTED_TARGETS})

## coordinated_motion_controllers
add_library(${PROJECT_NAME}
  src/coordinated_controller_base.cpp
  src/pose_controller.cpp
  src/axially_symmetric_controller.cpp
  src/positioner_controller.cpp
  src/positioner_objectives/positioner_objective_plugin.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} positioner_objectives
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    coordinated_controller_plugins.xml
    positioner_objective_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
