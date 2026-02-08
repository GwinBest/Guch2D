file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "*.cpp" "*.hpp")

add_executable(${PROJECT_NAME}-Demo ${SOURCES})
enable_asan(${PROJECT_NAME}-Demo)

target_include_directories(${PROJECT_NAME}-Demo PUBLIC "${CMAKE_SOURCE_DIR}/Src" "${CMAKE_SOURCE_DIR}/Demo")

# SFML
include(FetchContent)
FetchContent_Declare(
        SFML
        GIT_REPOSITORY https://github.com/SFML/SFML
        GIT_TAG 3.0.2
)
FetchContent_MakeAvailable(SFML)

target_link_libraries(${PROJECT_NAME}-Demo PRIVATE SFML::Graphics ${PROJECT_NAME})
