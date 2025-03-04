cmake_minimum_required(VERSION 3.10)
project(eo_as_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_include_directories(test_interface PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_interface PRIVATE
    eo_as_interface_lib
    eo_as_src
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_interface PRIVATE -Wall -Wextra -Wpedantic)
endif()