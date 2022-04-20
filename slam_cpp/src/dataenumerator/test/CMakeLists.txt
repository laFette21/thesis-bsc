set(target_name test_dataenumerator)

include_directories(${CMAKE_SOURCE_DIR}/src/misc)
include_directories(..)

add_executable(${target_name}
    ../DataEnumerator.cpp
    TestDataEnumerator.cpp
)

configure_file(empty.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(input1.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(input2.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(input3.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

add_test(${target_name}
    ${CMAKE_CURRENT_BINARY_DIR}/${target_name}
)

set_tests_properties(${target_name}
    PROPERTIES
    LABELS "cpp;dataenumerator"
)
