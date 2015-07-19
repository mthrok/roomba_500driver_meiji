cmake_minimum_required(VERSION 2.8.3)
project(create2_teleop)

find_package(catkin REQUIRED COMPONENTS
  create2_msgs
  joy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_drive src/joystick_drive.cpp)

add_dependencies(joystick_drive
  create2_msgs_generate_messages_cpp
)

target_link_libraries(joystick_drive
  ${catkin_LIBRARIES}
)
