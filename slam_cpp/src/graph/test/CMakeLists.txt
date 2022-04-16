set(target_name test_graph)

include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(..)

add_executable(${target_name}
    TestGraph.cpp
)

target_link_libraries(${target_name}
    ceres
)

add_test(${target_name}
    ${CMAKE_CURRENT_BINARY_DIR}/${target_name}
)

set_tests_properties(${target_name}
    PROPERTIES
    LABELS "cpp;graph"
)
