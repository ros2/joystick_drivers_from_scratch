cmake_minimum_required(VERSION 3.5)

project(joy)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS})

# for now, this program only exists for Linux hosts, since it calls into
# the underlying joystick driver provided by the Linux kernel.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  add_executable(joy_node joy_node_linux.cpp)
  target_link_libraries(joy_node ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES})
  install(TARGETS joy_node DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()

# this cross-platform node just listens to the joy topic and prints it
add_executable(joy_printer joy_printer.cpp)
target_link_libraries(joy_printer ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES})
install(TARGETS joy_printer DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

