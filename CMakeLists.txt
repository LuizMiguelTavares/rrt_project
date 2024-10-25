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
  filters
)

# Add the path to the custom Findmlpack.cmake script
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt_project
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/rrt_project
  include/header_lib
  include/external
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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

add_library(potential_lib
            src/potential.cpp)
    
# Executables
add_executable(differential_control nodes/differential_control.cpp)
target_link_libraries(differential_control ${catkin_LIBRARIES})

add_executable(differential_control_plot nodes/differential_control_plot.cpp)
target_link_libraries(differential_control_plot ${catkin_LIBRARIES})

add_executable(omni_control nodes/omni_control.cpp)
target_link_libraries(omni_control ${catkin_LIBRARIES})

add_executable(local_map_lidar nodes/local_map_lidar.cpp)
target_link_libraries(local_map_lidar ${catkin_LIBRARIES})

add_executable(merge_local_maps nodes/merge_local_maps.cpp)
target_link_libraries(merge_local_maps ${catkin_LIBRARIES})

add_executable(global_rrt nodes/global_rrt.cpp)
target_link_libraries(global_rrt ${catkin_LIBRARIES} motion_planning_lib)

add_executable(global_rrt_star nodes/global_rrt_star.cpp)
target_link_libraries(global_rrt_star ${catkin_LIBRARIES} rrt_star_lib)

add_executable(local_rrt nodes/local_rrt.cpp)
target_link_libraries(local_rrt ${catkin_LIBRARIES} motion_planning_lib)

add_executable(obstacle_avoidance nodes/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES} potential_lib)

add_executable(obstacle_avoidance_plot nodes/obstacle_avoidance_plot.cpp)
target_link_libraries(obstacle_avoidance_plot ${catkin_LIBRARIES} potential_lib)

add_executable(discretized_rectangle nodes/discretized_rectangle.cpp)
target_link_libraries(discretized_rectangle ${catkin_LIBRARIES}) 