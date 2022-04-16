set(target_name graph)

add_subdirectory(test)

include_directories(${CMAKE_SOURCE_DIR}/src)

add_library(${target_name}
    STATIC
    Graph.cpp
)

target_include_directories(${target_name}
    INTERFACE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(${target_name}
    ceres
)
