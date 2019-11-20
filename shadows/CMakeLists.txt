project(SHADOWS)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_STANDARD 14)

include_directories(include)

set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -g3 -Wno-deprecated")
set(GCC_COVERAGE_LINK_FLAGS    "-Iinclude -framework OpenGL")

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_definitions(-D_DEBUG)


set(SOURCES src/main.cpp src/shader.cpp src/vertexbuffer.cpp src/vertexbufferlayout.cpp src/mesh.cpp src/indexbuffer.cpp src/camera.cpp src/scene.cpp src/loadstb_image.cpp src/r_floor.cpp src/r_rabbit.cpp src/r_lightmarker.cpp src/r_objmodel.cpp)
set(GLEW ${CMAKE_CURRENT_SOURCE_DIR}/lib/glew/libGLEW.2.1.0.dylib)
set(GLFW ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw3/libglfw.dylib)
set(GLFW3 ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw3/libglfw.3.dylib)
set(SDL2 ${CMAKE_CURRENT_SOURCE_DIR}/lib/SDL2/libSDL2.dylib)
set(LIBRARIES ${GLEW} ${GLFW} ${GLFW3} )


add_executable(shadows ${SOURCES})
target_link_libraries(shadows ${LIBRARIES})