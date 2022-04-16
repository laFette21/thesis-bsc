set(target_name dataenumerator)

add_subdirectory(test)

include_directories(${CMAKE_SOURCE_DIR}/src/misc)

add_library(${target_name}
    STATIC
    DataEnumerator.cpp
)

target_include_directories(${target_name}
    INTERFACE
    ${CMAKE_SOURCE_DIR}/src/misc
)

target_link_libraries(${target_name}
    ceres
)
