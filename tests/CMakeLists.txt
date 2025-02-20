cmake_minimum_required(VERSION 3.10)
project(eo_as_test)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_interface test_eo_as_interface.cpp)

include_directories(test_interface PRIVATE 
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/../lib 
)

add_library(eo_as_interface_lib STATIC 
    ${CMAKE_SOURCE_DIR}/../lib/eo_as_interface_lib.cpp
)

target_link_libraries(test_interface PRIVATE eo_as_interface_lib)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_interface PRIVATE -Wall -Wextra -Wpedantic)
endif()
