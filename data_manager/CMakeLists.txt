project(key_value_data_manager)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})

set(SRC ${SRC} )

add_executable(key_value_data_manager main.cc ${SRC})
# target_link_directories(key_value_data_manager ${Eigen3_LIBRARIES})