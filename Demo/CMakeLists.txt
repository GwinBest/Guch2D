project(${PROJECT_NAME}-Demo)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/Demo/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

# Add sfml
include(FetchContent)
FetchContent_Declare(
        SFML
        GIT_REPOSITORY https://github.com/SFML/SFML
        GIT_TAG 3.0.2
)
FetchContent_MakeAvailable(SFML)
target_link_libraries(${PROJECT_NAME} PRIVATE SFML::Graphics)
