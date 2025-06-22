cmake_minimum_required(VERSION 3.19)

project(Guch2D LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set(imgui_src_path "${CMAKE_SOURCE_DIR}/ThirdParty/imgui")

# set(glfw_src_path "${CMAKE_SOURCE_DIR}/ThirdParty/glfw")

set(mathfu_src_path "${CMAKE_SOURCE_DIR}/ThirdParty/mathfu")

# # glfw
# set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs")
# set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs")
# set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation")
# set(GLFW_INSTALL OFF CACHE BOOL "Generate installation target") 
# add_subdirectory(${glfw_src_path} glfw_binary)

# glad
# add_library(glad STATIC ${glfw_src_path}/deps/glad_gl.c)
# target_include_directories(glad PUBLIC ${glfw_src_path}/deps/)

# # imgui with glfw + opengl3 backend
# add_library(imgui STATIC
#    ${imgui_src_path}/imgui.cpp
#    ${imgui_src_path}/imgui_demo.cpp
#    ${imgui_src_path}/imgui_draw.cpp
#    ${imgui_src_path}/imgui_tables.cpp
#    ${imgui_src_path}/imgui_widgets.cpp
#    ${imgui_src_path}/backends/imgui_impl_glfw.cpp
#    ${imgui_src_path}/backends/imgui_impl_opengl3.cpp
# )

# target_include_directories(imgui PUBLIC 
#    ${imgui_src_path}
#    ${imgui_src_path}/backends
# )

#target_compile_definitions(imgui PUBLIC IM_VEC2_CLASS_EXTRA)
# target_link_libraries(imgui glfw glad)

# arkanoid
file(GLOB srcs
   "Src/*.cpp"
   "Src/Dynamics/*.cpp" 
   "Src/Collisions/*.cpp" 
)

include_directories(
	${CMAKE_SOURCE_DIR}/src
)

# find_package(OpenGL REQUIRED)

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
# target_link_libraries(${CMAKE_PROJECT_NAME} glad imgui ${OPENGL_LIBRARIES})