cmake_minimum_required(VERSION 2.8.3)
project(psmove_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)
find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../../../include")
include_directories("../../../build")

link_directories("../../../build")

add_executable(psmove_ros main.cpp ros_move_interface.h)
target_link_libraries(psmove_ros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} psmoveapi psmoveapi_tracker)

