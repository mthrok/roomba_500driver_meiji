cmake_minimum_required(VERSION 2.8.3)
project(create2_driver)

find_package(catkin REQUIRED COMPONENTS
  create2_msgs
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS    include
  LIBRARIES       create2_driver
  CATKIN_DEPENDS  create2_msgs roscpp tf
  DEPENDS         system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(create2_driver
  src/${PROJECT_NAME}/create2sci.cpp
  src/${PROJECT_NAME}/serial.cpp
)

add_executable(create2_node
  src/create2_node.cpp
)

add_dependencies(create2_node
  create2_msgs_generate_messages_cpp
)

target_link_libraries(create2_node
  create2_driver
  ${catkin_LIBRARIES}
)
