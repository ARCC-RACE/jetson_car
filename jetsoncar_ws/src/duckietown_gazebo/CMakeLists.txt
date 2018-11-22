cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(duckietown_gazebo)

# Find packages

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS 
#  duckietown_msgs 
  )
find_package(Protobuf REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    gazebo_ros
    duckietown_msgs
  DEPENDS
    gazebo
    SDF
  LIBRARIES
#    duckie_ros_diff_drive
)


include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  /opt/ros/indigo/lib
  ${GAZEBO_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Create libraries and executables

#add_library(animated_box SHARED src/animated_box.cc)
#add_dependencies(animated_box duckietown_msgs_generate_messages_cpp $(catkin_EXPORTED_TARGETS))
#target_link_libraries(animated_box ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

#add_library(animated_box_south SHARED src/animated_box_south.cc)
#add_dependencies(animated_box_south duckietown_msgs_generate_messages_cpp $(catkin_EXPORTED_TARGETS))
#target_link_libraries(animated_box_south ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(integrated_main src/integrated_main.cc)
#add_dependencies(integrated_main duckietown_msgs_generate_messages_cpp $(catkin_EXPORTED_TARGETS))
#target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)


#add_executable(independent_listener src/independent_listener.cc)
#add_dependencies(independent_listener duckietown_msgs_generate_messages_cpp $(catkin_EXPORTED_TARGETS))
#target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

#add_executable(light_publisher  src/publisher.cc)
#add_dependencies(light_publisher duckietown_msgs_generate_messages_cpp $(catkin_EXPORTED_TARGETS))
#target_link_libraries(light_publisher  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

#add_library(duckie_ros_diff_drive src/duckie_ros_diff_drive.cpp)
#add_dependencies(duckie_ros_diff_drive duckietown_msgs_generate_messages_cpp $(catkin_EXPORTED_TARGETS))
#target_link_libraries(duckie_ros_diff_drive gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

