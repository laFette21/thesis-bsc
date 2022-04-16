set(target_name test_misc)

include_directories(..)

add_executable(${target_name}
    TestTypes.cpp
    TestUtils.cpp
)

target_link_libraries(${target_name}
    ceres
)

add_test(${target_name}
    ${CMAKE_CURRENT_BINARY_DIR}/${target_name}
)

set_tests_properties(${target_name}
    PROPERTIES
    LABELS "cpp;types;utils"
)
