cmake_minimum_required(VERSION 3.19)

project(Guch2D LANGUAGES C CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DEMO "Build demo executable" OFF)
option(BUILD_TEST "Build tests" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer" ON)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" ON)
option(ENABLE_LSAN "Enable LeakSanitizer" ON)

include(CheckCXXCompilerFlag)
include(CheckLinkerFlag)

set(PROJECT_SANITIZER_COMPILE_FLAGS "")
set(PROJECT_SANITIZER_LINK_FLAGS "")

macro(append_sanitizer_if_supported sanitizer_name sanitizer_flag)
    string(MAKE_C_IDENTIFIER "${sanitizer_flag}" sanitizer_flag_id)

    if (DEFINED CMAKE_TRY_COMPILE_TARGET_TYPE)
        set(_saved_try_compile_target_type "${CMAKE_TRY_COMPILE_TARGET_TYPE}")
        set(_saved_try_compile_target_type_is_set TRUE)
    else ()
        set(_saved_try_compile_target_type_is_set FALSE)
    endif ()

    set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
    check_cxx_compiler_flag("${sanitizer_flag}" "HAS_CXX_FLAG_${sanitizer_flag_id}")
    if (_saved_try_compile_target_type_is_set)
        set(CMAKE_TRY_COMPILE_TARGET_TYPE "${_saved_try_compile_target_type}")
    else ()
        unset(CMAKE_TRY_COMPILE_TARGET_TYPE)
    endif ()

    check_linker_flag(CXX "${sanitizer_flag}" "HAS_LINKER_FLAG_${sanitizer_flag_id}")

    if (HAS_CXX_FLAG_${sanitizer_flag_id} AND HAS_LINKER_FLAG_${sanitizer_flag_id})
        list(APPEND PROJECT_SANITIZER_COMPILE_FLAGS "${sanitizer_flag}")
        list(APPEND PROJECT_SANITIZER_LINK_FLAGS "${sanitizer_flag}")
    else ()
        message(WARNING
                "${sanitizer_name} requested, but '${sanitizer_flag}' is not fully supported by "
                "${CMAKE_CXX_COMPILER_ID} (${CMAKE_CXX_COMPILER}). Skipping ${sanitizer_name}."
        )
    endif ()
endmacro()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    if (ENABLE_ASAN)
        append_sanitizer_if_supported("ASan" "-fsanitize=address")
    endif ()
    if (ENABLE_UBSAN)
        append_sanitizer_if_supported("UBSan" "-fsanitize=undefined")
    endif ()
    if (ENABLE_LSAN)
        append_sanitizer_if_supported("LSan" "-fsanitize=leak")
    endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if (ENABLE_UBSAN)
        message(WARNING "UBSan is not supported by the MSVC toolchain and will be ignored.")
    endif ()
    if (ENABLE_LSAN)
        message(WARNING "LSan is not supported by the MSVC toolchain and will be ignored.")
    endif ()
endif ()

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

function(enable_sanitizers target_name)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        if (NOT PROJECT_SANITIZER_COMPILE_FLAGS)
            return()
        endif ()

        foreach (sanitizer_flag IN LISTS PROJECT_SANITIZER_COMPILE_FLAGS)
            target_compile_options(${target_name} PRIVATE
                    $<$<CONFIG:Debug>:${sanitizer_flag}>
                    $<$<CONFIG:RelWithDebInfo>:${sanitizer_flag}>
            )
        endforeach ()

        foreach (sanitizer_flag IN LISTS PROJECT_SANITIZER_LINK_FLAGS)
            target_link_options(${target_name} PRIVATE
                    $<$<CONFIG:Debug>:${sanitizer_flag}>
                    $<$<CONFIG:RelWithDebInfo>:${sanitizer_flag}>
            )
        endforeach ()

        if ("-fsanitize=address" IN_LIST PROJECT_SANITIZER_COMPILE_FLAGS
                OR "-fsanitize=leak" IN_LIST PROJECT_SANITIZER_COMPILE_FLAGS)
            target_compile_options(${target_name} PRIVATE
                    $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
                    $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
            )
        endif ()
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        if (NOT ENABLE_ASAN)
            return()
        endif ()

        target_compile_options(${target_name} PRIVATE
                $<$<CONFIG:Debug>:/fsanitize=address>
                $<$<CONFIG:RelWithDebInfo>:/fsanitize=address>
        )
        copy_msvc_asan_runtime(${target_name})
    endif ()
endfunction()

function(enable_asan target_name)
    enable_sanitizers(${target_name})
endfunction()

function(enable_project_warnings target_name)
    target_compile_options(${target_name} PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
            $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wconversion -Wshadow -Werror>
    )
endfunction()

add_subdirectory(Src)

if (BUILD_DEMO)
    add_subdirectory(Demo)
endif ()

if (BUILD_TEST)
    add_subdirectory(Test)
endif ()
