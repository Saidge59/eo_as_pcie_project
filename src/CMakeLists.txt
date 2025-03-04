cmake_minimum_required(VERSION 3.10)
project(eo_as_src LANGUAGES CXX)

add_library(eo_as_src STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/config_reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tcp_commands_server.cpp
)

target_include_directories(eo_as_src PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(eo_as_src PUBLIC eo_as_interface_lib)