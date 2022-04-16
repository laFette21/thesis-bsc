cmake_minimum_required(VERSION 3.10)
project(GraphSLAM)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(
    -O3
    -Wall
    -g
)

list(APPEND
    CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake_modules
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

enable_testing()

add_subdirectory(src)
