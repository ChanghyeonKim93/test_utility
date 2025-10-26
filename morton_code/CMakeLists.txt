project(morton_code)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(example_morton_code main.cc)
