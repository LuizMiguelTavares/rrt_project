cmake_minimum_required(VERSION 3.0.2)
project(rrt_project)
set(CMAKE_CXX_STANDARD 20)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf
  sensor_msgs
  obstacle_avoidance_drone_follower
  nav_msgs
  geometry_msgs
  map_msgs
  message_generation
)

add_service_files(
  FILES
  AddTwoInts.srv
  ClusterObstacles.srv
)

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

# add_library(image_transformation
#   src/image_transformation.cpp
# )

add_library(motion_planning_lib INTERFACE)
target_include_directories(motion_planning_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/header_lib)
target_link_libraries(motion_planning_lib INTERFACE ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

add_library(rrt_star_lib
  src/rrt_star_new.cpp
)

target_link_libraries(rrt_star_lib
  ${OpenCV_LIBS} motion_planning_lib
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rrt_project.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# add_executable(rrt_path_node src/nodes/rrt_path.cpp)
# add_dependencies(rrt_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rrt_path_node ${catkin_LIBRARIES} motion_planning_lib)

# add_executable(rrt_simple src/rrt_simple_map.cpp src/image_transformation.cpp)
# target_link_libraries(rrt_simple ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(mapping_node src/nodes/mapping.cpp)
# target_link_libraries(mapping_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(publish_robot_potential_points src/nodes/robot_points.cpp)
target_link_libraries(publish_robot_potential_points ${catkin_LIBRARIES})

add_executable(velocity_publisher src/nodes/velocity_calculator_node.cpp)
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})

add_executable(local_map src/nodes/local_map.cpp)
target_link_libraries(local_map ${catkin_LIBRARIES})

add_executable(merge_local_map src/nodes/merge_local_map.cpp)
target_link_libraries(merge_local_map ${catkin_LIBRARIES})

add_executable(global_rrt src/nodes/global_rrt.cpp)
target_link_libraries(global_rrt ${catkin_LIBRARIES} motion_planning_lib)

add_executable(global_rrt_star src/nodes/global_rrt_star.cpp)
target_link_libraries(global_rrt_star ${catkin_LIBRARIES} rrt_star_lib)

add_executable(local_rrt src/nodes/local_rrt.cpp)
target_link_libraries(local_rrt ${catkin_LIBRARIES} motion_planning_lib)

# add_executable(test_refactor src/nodes/test_refactor.cpp)
# target_link_libraries(test_refactor ${catkin_LIBRARIES} motion_planning_lib ${OpenCV_LIBS})

# add_executable(test_rrt_ptr src/test_rrt_ptr.cpp)
# target_link_libraries(test_rrt_ptr ${catkin_LIBRARIES} motion_planning_lib image_transformation ${OpenCV_LIBS})

# add_executable(minimal_local_rrt src/nodes/minimal_local_rrt.cpp)
# target_link_libraries(minimal_local_rrt ${catkin_LIBRARIES} motion_planning_lib ${OpenCV_LIBS})

# add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# # find_package(MLPACK REQUIRED)
# find_package(Armadillo REQUIRED)

# include_directories(${ARMADILLO_INCLUDE_DIRS})
# add_executable(ClusterVisualization test/hierarchical_clustering.cpp)
# target_link_libraries(ClusterVisualization ${OpenCV_LIBS} ${MLPACK_LIBRARIES} ${ARMADILLO_LIBRARIES})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rrt_project.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
