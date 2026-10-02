# Минимальная версия CMake, которая нужна проекту
cmake_minimum_required(VERSION 3.20)

# Имя проекта и язык
project(Pin3D LANGUAGES CXX)

# Стандарт C++ (20 — современный, есть и в GCC, и в Visual Studio на Windows)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Найти установленную библиотеку raylib (окно, 3D, ввод)
find_package(raylib REQUIRED)

# Собрать программу pin3d из этих файлов
add_executable(pin3d
    src/main.cpp
)

# Подключить raylib к нашей программе
target_link_libraries(pin3d PRIVATE raylib)

# Включить предупреждения компилятора (как -Wall)
if(MSVC)
    target_compile_options(pin3d PRIVATE /W4)
else()
    target_compile_options(pin3d PRIVATE -Wall -Wextra)
endif()
