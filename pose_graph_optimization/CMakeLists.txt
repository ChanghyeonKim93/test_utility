project(pose_graph_optimization)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SRC ${SRC} 
  optimizer/time_checker.cc
  optimizer/summary_reporter.cc
  optimizer/pose_graph_optimizer.cc
  optimizer/pose_graph_optimizer_ceres_solver.cc
)

add_executable(example_simple examples/main_simple.cc ${SRC})
target_link_libraries(example_simple 
  ${Eigen3_LIBRARIES} 
  ${CERES_LIBRARIES})
