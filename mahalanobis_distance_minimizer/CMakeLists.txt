cmake_minimum_required(VERSION 3.9)
project(mahalanobis_distance_minimizer)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set(
  INCLUDE_DIR 
  ${INCLUDE_DIR} 
  ${CMAKE_CURRENT_BINARY_DIR}
  .
)

include_directories(
  ${INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

set(SRC ${SRC}
  time_checker.cc
  optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer_ceres.cc
)

add_executable(${CMAKE_PROJECT_NAME}_main
  main.cc ${SRC})
target_link_libraries(${PROJECT_NAME}_main PRIVATE ${CERES_LIBRARIES})

add_executable(simple_optimization_test  tests/simple_optimization_test.cc ${SRC})
target_link_libraries(simple_optimization_test PRIVATE ${CERES_LIBRARIES})