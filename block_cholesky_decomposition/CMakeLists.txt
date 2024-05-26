project(block_cholesky_decomposition)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})

set(SRC ${SRC} 
  timer.cpp
  thirdparty/csparse.cc
)

add_executable(block_cholesky_decomposition main.cc ${SRC})

add_executable(example_csparse_with_eigen examples/example_csparse_with_eigen.cc ${SRC})
target_link_libraries(example_csparse_with_eigen 
  ${Eigen3_LIBRARIES} ${CERES_LIBRARIES})

add_executable(example_eigen_sparse_cholesky examples/example_eigen_sparse_cholesky.cc ${SRC})
target_link_libraries(example_eigen_sparse_cholesky 
  ${Eigen3_LIBRARIES} ${CERES_LIBRARIES})