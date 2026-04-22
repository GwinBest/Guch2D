file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "*.cpp"
        "*.hpp"
)

add_library(${PROJECT_NAME} ${SOURCES})
if (ENABLE_CLANG_TIDY AND PROJECT_CXX_CLANG_TIDY)
    set_target_properties(${PROJECT_NAME} PROPERTIES CXX_CLANG_TIDY "${PROJECT_CXX_CLANG_TIDY}")
endif ()
enable_asan(${PROJECT_NAME})
enable_project_warnings(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# glm
include(FetchContent)
FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)
target_link_libraries(${PROJECT_NAME} PUBLIC glm::glm)
