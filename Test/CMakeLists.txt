# Add google test
include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG v1.17.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Keep ASan annotation settings consistent across all linked test libraries.
foreach (gtest_target IN ITEMS gtest gtest_main gmock gmock_main)
    if (TARGET ${gtest_target})
        enable_asan(${gtest_target})
    endif ()
endforeach ()

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/Test/*.cpp")

add_executable(${PROJECT_NAME}-Test ${SOURCES})
enable_asan(${PROJECT_NAME}-Test)
enable_project_warnings(${PROJECT_NAME}-Test)

target_link_libraries(${PROJECT_NAME}-Test PRIVATE gtest gtest_main ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}-Test
        PRIVATE
        ${gtest_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/Src
)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}-Test)
