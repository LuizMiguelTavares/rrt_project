cmake_minimum_required(VERSION 3.0.2)
project(rrt_project)
set(CMAKE_CXX_STANDARD 20)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf
  sensor_msgs
  nav_msgs
  geometry_msgs
  map_msgs
  message_generation
)

# add_service_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt_project
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/rrt_project
  include/header_lib
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Libraries
add_library(motion_planning_lib INTERFACE)
target_include_directories(motion_planning_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/header_lib)
target_link_libraries(motion_planning_lib INTERFACE ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

add_library(rrt_star_lib
  src/rrt_star.cpp
)

target_link_libraries(rrt_star_lib
  ${OpenCV_LIBS} motion_planning_lib
)

# Executables
add_executable(publish_robot_potential_points nodes/robot_points.cpp)
target_link_libraries(publish_robot_potential_points ${catkin_LIBRARIES})

add_executable(local_map nodes/local_map.cpp)
target_link_libraries(local_map ${catkin_LIBRARIES})

add_executable(merge_local_map nodes/merge_local_map.cpp)
target_link_libraries(merge_local_map ${catkin_LIBRARIES})

add_executable(global_rrt nodes/global_rrt.cpp)
target_link_libraries(global_rrt ${catkin_LIBRARIES} motion_planning_lib)

add_executable(global_rrt_star nodes/global_rrt_star.cpp)
target_link_libraries(global_rrt_star ${catkin_LIBRARIES} rrt_star_lib)

add_executable(local_rrt nodes/local_rrt.cpp)
target_link_libraries(local_rrt ${catkin_LIBRARIES} motion_planning_lib)