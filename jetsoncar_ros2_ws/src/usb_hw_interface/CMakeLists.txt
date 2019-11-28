cmake_minimum_required(VERSION 3.5)
project(usb_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(usb_hw_interface_node src/hw_interface.cpp src/serial.cpp src/unix.cpp src/racecar_arduino_com.cpp)
ament_target_dependencies(usb_hw_interface_node rclcpp ackermann_msgs sensor_msgs geometry_msgs)

install(TARGETS
  usb_hw_interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()