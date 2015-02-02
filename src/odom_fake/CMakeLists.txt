cmake_minimum_required(VERSION 2.8.3)
project(odom_fake)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs tf)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES sensor_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_fake src/odom.cpp src/SysFsHelper.cpp)
target_link_libraries(odom_fake ${catkin_LIBRARIES})
add_dependencies(odom_fake odom_fake_generate_messages_cpp)
