cmake_minimum_required(VERSION 3.20)

project(Guch2D VERSION 1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IMGUI_SRC_PATH "${CMAKE_SOURCE_DIR}/thirdParty/imgui")

set(GLFW_SRC_PATH "${CMAKE_SOURCE_DIR}/thirdParty/glfw")

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE BOOL "Generate installation target") 
add_subdirectory(${GLFW_SRC_PATH} glfw_binary)

add_library(glad STATIC ${GLFW_SRC_PATH}/deps/glad_gl.c)
target_include_directories(glad PUBLIC ${GLFW_SRC_PATH}/deps/)

add_library(imgui STATIC
   ${IMGUI_SRC_PATH}/imgui.cpp
   ${IMGUI_SRC_PATH}/imgui_demo.cpp
   ${IMGUI_SRC_PATH}/imgui_draw.cpp
   ${IMGUI_SRC_PATH}/imgui_tables.cpp
   ${IMGUI_SRC_PATH}/imgui_widgets.cpp
   ${IMGUI_SRC_PATH}/backends/imgui_impl_glfw.cpp
   ${IMGUI_SRC_PATH}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC 
   ${IMGUI_SRC_PATH}
   ${IMGUI_SRC_PATH}/backends
)

target_link_libraries(imgui glfw glad)

file(GLOB SRCS
   "src/*.cpp"
   "src/gameObjects/*.cpp" 
   "src/scenes/*.cpp"
)

include_directories(
	${CMAKE_SOURCE_DIR}/src
)

find_package(OpenGL REQUIRED)

add_executable(${CMAKE_PROJECT_NAME} ${SRCS})

target_link_libraries(${CMAKE_PROJECT_NAME} glad imgui ${OPENGL_LIBRARIES})