set(target_name test_dataenumerator)

include_directories(${CMAKE_SOURCE_DIR}/src/misc)
include_directories(..)

add_executable(${target_name}
    ../DataEnumerator.cpp
    TestDataEnumerator.cpp
)

add_test(${target_name}
    ${CMAKE_CURRENT_BINARY_DIR}/${target_name}
)

set_tests_properties(${target_name}
    PROPERTIES
    LABELS "cpp;dataenumerator"
)
