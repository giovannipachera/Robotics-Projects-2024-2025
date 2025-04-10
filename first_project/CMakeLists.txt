cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Trova catkin e pacchetti richiesti
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  message_filters
)

## Service
add_service_files(
  FILES
  GpsToOdom.srv
)

## Custom message
add_message_files(
  FILES
  Sector_time.msg
)

## Genera i messaggi
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

## Dichiarazione package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs sensor_msgs tf message_filters
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Nodi eseguibili
add_executable(odometer src/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})
add_dependencies(odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})
add_dependencies(gps_to_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})
add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sector_times src/sector_times.cpp)
target_link_libraries(sector_times ${catkin_LIBRARIES})
add_dependencies(sector_times ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
