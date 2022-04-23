set(target_name slam)

add_subdirectory(dataenumerator)
add_subdirectory(graph)
add_subdirectory(misc)

add_executable(${target_name}
    main.cpp
)

target_include_directories(${target_name}
    PUBLIC
    dataenumerator
    graph
    misc
)

target_link_libraries(${target_name}
    dataenumerator
    graph
)
