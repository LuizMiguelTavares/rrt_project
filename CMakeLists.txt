# cmake_minimum_required(VERSION 3.10)

# # Set the project name
# project(RRT_Projects)

# add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# # Specify the C++ standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
# include_directories(/usr/include)
# link_directories(/usr/lib/x86_64-linux-gnu)

# # Find OpenCV and OpenMP
# find_package(OpenCV REQUIRED)
# find_package(OpenMP REQUIRED)

# # Optionally, enable profiling with gprof for Debug builds
# # set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg")
# # set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -pg")

# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# # Add your source files here
# add_executable(RRT_simple src/rrt_simple_map.cpp src/image_transformation.cpp)
# add_executable(RRT_kynematics src/rrt_kynematics.cpp)
# add_executable(RRT_star src/main_rrt_star.cpp src/rrt_star.cpp src/image_transformation.cpp)
# add_executable(QTree_test src/test_quad_tree.cpp)

# # Link with OpenCV libraries
# target_link_libraries(RRT_simple ${OpenCV_LIBS})
# target_link_libraries(RRT_kynematics ${OpenCV_LIBS})
# target_link_libraries(RRT_star ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
# target_link_libraries(QTree_test ${OpenCV_LIBS})

# include_directories(${PROJECT_SOURCE_DIR}/Include)

cmake_minimum_required(VERSION 3.10)

# Set the project name
project(RRT_Projects)

add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
include_directories(/usr/include)
link_directories(/usr/lib/x86_64-linux-gnu)

# Find OpenCV and OpenMP
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# Debug-specific flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Optionally, enable profiling with gprof for Debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -pg")

# Release-specific flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Add your source files here
add_executable(RRT_simple src/rrt_simple_map.cpp src/image_transformation.cpp)
add_executable(RRT_kynematics src/rrt_kynematics.cpp)
add_executable(RRT_star src/main_rrt_star.cpp src/rrt_star.cpp src/image_transformation.cpp)
add_executable(QTree_test src/test_quad_tree.cpp)

# Link with OpenCV libraries
target_link_libraries(RRT_simple ${OpenCV_LIBS})
target_link_libraries(RRT_kynematics ${OpenCV_LIBS})
target_link_libraries(RRT_star ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
target_link_libraries(QTree_test ${OpenCV_LIBS})

include_directories(${PROJECT_SOURCE_DIR}/Include)