project(SHADOWS)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_STANDARD 14)

set(SOURCES main.cpp)
include_directories(include)

set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -g3 -Wno-deprecated")
set(GCC_COVERAGE_LINK_FLAGS    "-Iinclude -framework OpenGL")

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_definitions(-D_DEBUG)


add_executable(shadows src/main.cpp src/shader.cpp src/vertexbuffer.cpp src/vertexbufferlayout.cpp src/mesh.cpp src/indexbuffer.cpp src/camera.cpp src/scene.cpp src/loadstb_image.cpp src/r_floor.cpp src/r_rabbit.cpp)
target_link_libraries(shadows ${CMAKE_CURRENT_SOURCE_DIR}/lib/glew/libGLEW.2.1.0.dylib ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw3/libglfw.dylib ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw3/libglfw.3.dylib)