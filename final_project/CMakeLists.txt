cmake_minimum_required(VERSION 2.8.3)
project(final_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  visualization_msgs
  dynamic_reconfigure
  sensor_msgs
  std_msgs
  geometry_msgs
  
)

generate_dynamic_reconfigure_options(
  cfg/Waypoint.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#################
## Test Script ##
#################

# add_executable(waypoint src/waypoint.cpp)
# target_link_libraries(waypoint ${catkin_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(target src/target.cpp)
target_link_libraries(target ${catkin_LIBRARIES})