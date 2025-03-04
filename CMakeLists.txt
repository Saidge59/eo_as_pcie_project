cmake_minimum_required(VERSION 3.10)
project(eo_as_project LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(eo_as_pcie_driver)
add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(tests)

add_custom_target(build_all
    DEPENDS eo_as_driver test_interface
    COMMENT "Building both kernel module and user-space components"
)

add_custom_target(clean_all
    COMMAND ${CMAKE_MAKE_PROGRAM} clean
    COMMAND ${CMAKE_MAKE_PROGRAM} eo_as_clean
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Cleaning both user-space and kernel module artifacts"
)