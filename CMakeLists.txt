cmake_minimum_required(VERSION 3.19)

project(Guch2D LANGUAGES C CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DEMO "Build demo executable" OFF)
option(BUILD_TEST "Build tests" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer" ON)

function(copy_msvc_asan_runtime target_name)
    if (NOT MSVC OR NOT ENABLE_ASAN)
        return()
    endif ()

    get_target_property(target_type ${target_name} TYPE)
    if (NOT target_type STREQUAL "EXECUTABLE")
        return()
    endif ()

    get_filename_component(msvc_bin_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    file(GLOB asan_runtime_dlls "${msvc_bin_dir}/clang_rt.asan*.dll")

    if (NOT asan_runtime_dlls)
        message(WARNING
                "ASan is enabled, but no clang_rt.asan*.dll was found in '${msvc_bin_dir}'. "
                "Tests may fail at runtime with 0xc0000135."
        )
        return()
    endif ()

    foreach (asan_runtime_dll IN LISTS asan_runtime_dlls)
        add_custom_command(
                TARGET ${target_name}
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${asan_runtime_dll}"
                "$<TARGET_FILE_DIR:${target_name}>"
                VERBATIM
        )
    endforeach ()
endfunction()

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
        copy_msvc_asan_runtime(${target_name})
    endif ()
endfunction()

add_subdirectory(Src)

if (BUILD_DEMO)
    add_subdirectory(Demo)
endif ()

if (BUILD_TEST)
    add_subdirectory(Test)
endif ()
