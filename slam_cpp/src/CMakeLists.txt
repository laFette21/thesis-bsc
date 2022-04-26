set(target_name slam)

add_subdirectory(dataenumerator)
add_subdirectory(graph)
add_subdirectory(matplotlibcpp)
add_subdirectory(misc)

add_executable(${target_name}
    main.cpp
)

target_include_directories(${target_name}
    PUBLIC
    dataenumerator
    graph
    matplotlibcpp
    misc
)

target_link_libraries(${target_name}
    dataenumerator
    graph
    matplotlibcpp
)
