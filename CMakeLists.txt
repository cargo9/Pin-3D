cmake_minimum_required(VERSION 3.20)

project(Pin3D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib REQUIRED)

add_executable(pin3d
    src/main.cpp
)

target_link_libraries(pin3d PRIVATE raylib)
target_compile_definitions(pin3d PRIVATE ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

if(MSVC)
    target_compile_options(pin3d PRIVATE /W4)
else()
    target_compile_options(pin3d PRIVATE -Wall -Wextra)
endif()
