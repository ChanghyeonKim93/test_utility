project(message_queue_pattern)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})

set(SRC ${SRC} )

add_executable(example_simple examples/main_simple.cc ${SRC})
add_executable(example_realistic_situation examples/main_realistic_situation.cc ${SRC})