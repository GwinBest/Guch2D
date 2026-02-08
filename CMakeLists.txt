cmake_minimum_required(VERSION 3.19)

project(Guch2D LANGUAGES C CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DEMO "Build demo executable" OFF)
option(BUILD_TEST "Build tests" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer" ON)

function(enable_asan target_name)
    if (NOT ENABLE_ASAN)
        return()
    endif ()

    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${target_name} PRIVATE
                $<$<CONFIG:Debug>:-fsanitize=address -fno-omit-frame-pointer>
                $<$<CONFIG:RelWithDebInfo>:-fsanitize=address -fno-omit-frame-pointer>
        )
        target_link_options(${target_name} PRIVATE
                $<$<CONFIG:Debug>:-fsanitize=address>
                $<$<CONFIG:RelWithDebInfo>:-fsanitize=address>
        )
    elseif (MSVC)
        target_compile_options(${target_name} PRIVATE
                $<$<CONFIG:Debug>:/fsanitize=address>
                $<$<CONFIG:RelWithDebInfo>:/fsanitize=address>
        )
    endif ()
endfunction()

add_subdirectory(Src)

if (BUILD_DEMO)
    add_subdirectory(Demo)
endif ()

if (BUILD_TEST)
    add_subdirectory(Test)
endif ()
