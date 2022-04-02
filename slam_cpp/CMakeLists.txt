cmake_minimum_required(VERSION 3.10)
project(SLAM)

set(CMAKE_CXX_STANDARD 20)
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set (CMAKE_CXX_FLAGS "-O3 -Wall")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

link_libraries(ceres)

file(GLOB SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable(slam ${SOURCE_FILES})
