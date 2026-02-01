cmake_minimum_required(VERSION 3.19)

project(Guch2D LANGUAGES C CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DEMO "Build demo executable" OFF)
option(BUILD_TEST "Build tests" OFF)

add_subdirectory(Src)

if (BUILD_DEMO)
    add_subdirectory(Demo)
endif ()

if (BUILD_TEST)
    add_subdirectory(Test)
endif ()
