cmake_minimum_required(VERSION 3.10)

# Set the project name
project(RRT_simple)
project(RRT_kynematics)
project(RRT_star)

add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# Add your source files here
# add_executable(RRT_simple src/rrt_test.cpp)
add_executable(RRT_simple src/rrt_simple_map.cpp src/image_transformation.cpp)
add_executable(RRT_kynematics src/rrt_kynematics.cpp)
add_executable(RRT_star src/main_rrt_star.cpp src/rrt_star.cpp src/image_transformation.cpp)

# Link with OpenCV libraries
target_link_libraries(RRT_simple ${OpenCV_LIBS})
target_link_libraries(RRT_kynematics ${OpenCV_LIBS})
target_link_libraries(RRT_star ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

include_directories(${PROJECT_SOURCE_DIR}/Include)