cmake_minimum_required(VERSION 3.9)
project(mahalanobis_distance_minimizer)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)

set(INCLUDE_DIR ${INCLUDE_DIR} 
  ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

set(SRC ${SRC}
  time_checker.cc
)

add_executable(${CMAKE_PROJECT_NAME}_main
  main.cc ${SRC})