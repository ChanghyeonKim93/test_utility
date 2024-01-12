cmake_minimum_required(VERSION 3.9)
project(scan_matching)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

# include_directories(${CMAKE_SOURCE_DIR})
set(SRC ${SRC}
  scan_matcher_analytic_solver.cc)

add_executable(example_scan_matcher_analytic_solver
 example/example_scan_matcher_analytic_solver.cc 
 ${SRC})
target_link_libraries(example_scan_matcher_analytic_solver PUBLIC ${OpenCV_LIBS})
target_include_directories(example_scan_matcher_analytic_solver PUBLIC ${OpenCV_LIBS} ${CMAKE_SOURCE_DIR})
