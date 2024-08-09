cmake_minimum_required(VERSION 3.0.2)
project(coordinated_motion_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/coordinated_motion_demo_rob1.py
  scripts/coordinated_motion_demo_rob2.py
  scripts/twist_decomposition_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
