file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "*.cpp")

add_executable(${PROJECT_NAME}-Demo ${SOURCES})

target_include_directories(${PROJECT_NAME}-Demo PUBLIC "${CMAKE_SOURCE_DIR}/Src")

# SFML
include(FetchContent)
FetchContent_Declare(
        SFML
        GIT_REPOSITORY https://github.com/SFML/SFML
        GIT_TAG 3.0.2
)
FetchContent_MakeAvailable(SFML)

target_link_libraries(${PROJECT_NAME}-Demo PRIVATE SFML::Graphics ${PROJECT_NAME})
