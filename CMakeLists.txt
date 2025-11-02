cmake_minimum_required(VERSION 3.19)

project(Guch2D LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(mathfu_src_path "${CMAKE_SOURCE_DIR}/ThirdParty/mathfu")

file(GLOB srcs
        "Src/*.cpp"
        "Src/Dynamics/*.cpp"
        "Src/Collisions/*.cpp"
)

include_directories(
        ${CMAKE_SOURCE_DIR}/src
)

include(FetchContent)
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 3.0.1
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(SFML)

add_executable(${CMAKE_PROJECT_NAME} ${srcs})

set_directory_properties(PROPERTIES VS_STARTUP_PROJECT ${CMAKE_PROJECT_NAME})

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE MATHFU_COMPILE_WITHOUT_SIMD_SUPPORT)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${mathfu_src_path}/include/)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE SFML::Graphics)
