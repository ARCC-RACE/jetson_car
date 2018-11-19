cmake_minimum_required(VERSION 3.5)
project(carHWComTest)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(carHWComTest main.cpp racecar_arduino_com.cpp serial.cpp unix.cpp)
