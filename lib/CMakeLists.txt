cmake_minimum_required(VERSION 3.10)
project(eo_as_interface_lib LANGUAGES CXX)

add_library(eo_as_interface_lib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/eo_as_interface_lib.cpp
)

target_include_directories(eo_as_interface_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)