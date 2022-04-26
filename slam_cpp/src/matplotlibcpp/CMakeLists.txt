set(target_name matplotlibcpp)

# Matplotlib
add_library(${target_name} INTERFACE)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

target_link_libraries(${target_name} INTERFACE
    Python3::Python
    Python3::Module
)

find_package(Python3 COMPONENTS NumPy)

if(Python3_NumPy_FOUND)
    target_link_libraries(${target_name} INTERFACE
        Python3::NumPy
    )
else()
    target_compile_definitions(${target_name} INTERFACE WITHOUT_NUMPY)
endif()
