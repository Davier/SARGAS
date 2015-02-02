cmake_minimum_required(VERSION 2.8.3)
project(sharp)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES sensor_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sharp src/sharp.cpp src/SysFsHelper.cpp src/ADC.cpp)
target_link_libraries(sharp ${catkin_LIBRARIES})
add_dependencies(sharp sharp_generate_messages_cpp)
