cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
  tf2_ros
  message_filters
  move_base_msgs
  actionlib
  roslib
)
## Package declaration
catkin_package(
  CATKIN_DEPENDS
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Executable nodes
add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(combine_laserscan src/combine_laserscan.cpp)
target_link_libraries(combine_laserscan ${catkin_LIBRARIES})

add_executable(static_tf_scan_back src/static_tf_scan_back.cpp)
target_link_libraries(static_tf_scan_back ${catkin_LIBRARIES})

add_executable(send_goals_from_csv src/send_goals_from_csv.cpp)
target_link_libraries(send_goals_from_csv ${catkin_LIBRARIES})



