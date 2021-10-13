cmake_minimum_required(VERSION 3.0.2)
project(robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  tf
  darknet_ros_msgs
  cv_bridge
  track_pkg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_node src/move/navigation.cpp
                               src/move/navigation_node.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES})

add_executable(detection_node src/detection/yolo_detection_core.cpp
                              src/detection/yolo_detection_node.cpp)
target_link_libraries(detection_node ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES})

add_executable(go_room_node src/go_room/go_room_core.cpp
                            src/go_room/go_room_node.cpp
                            src/people_tracking/tracking.cpp)

target_link_libraries(go_room_node ${catkin_LIBRARIES})

add_executable(tracking_node src/people_tracking/tracking.cpp
                             src/people_tracking/people_tracking_node.cpp)

target_link_libraries(tracking_node ${catkin_LIBRARIES}
                                    ${OpenCV_LIBRARIES})

add_executable(communication_node src/communication/communication_core.cpp
                                  src/communication/communication_node.cpp)

target_link_libraries(communication_node ${catkin_LIBRARIES})


