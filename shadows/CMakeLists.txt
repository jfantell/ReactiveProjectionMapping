project(SHADOWS)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build")
add_definitions(-D_DEBUG)

file(GLOB SOURCES "src/*.cpp")
include_directories(include)

if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()

if(LINUX)
    message("LINUX")

    find_library(GL libGL.so.1 HINTS /usr/lib/x86_64-linux-gnu REQUIRED)
    find_library(GLEW GLEW REQUIRED)
    find_library(GLFW libglfw.so.3 HINTS /usr/lib/x86_64-linux-gnu REQUIRED)
    find_library(REALSENSE2 realsense2 REQUIRED)
    find_library(SOIL2 soil2-debug HINTS /usr/lib)
    find_package(OpenCV REQUIRED)
    find_package(PCL 1.3 REQUIRED)

    # Ensure all libraries are found
    MESSAGE("STATUS ${GL} ${GLEW} ${GLFW} ${REALSENSE2} ${SOIL2} ${OpenCV_LIBS} ${PCL_LIBRARIES}")

    # Per PCL documentation
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})

    set(LIBRARIES ${SOIL2}
            ${GLEW}
            ${GLFW}
            ${REALSENSE2}
            ${GL}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${PDAL})

else()
    message("MAC OS")
    set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -g3 -Wno-deprecated")
    set(GCC_COVERAGE_LINK_FLAGS    "-Iinclude -framework OpenGL")

    set(GLEW ${CMAKE_CURRENT_SOURCE_DIR}/lib/glew/libGLEW.2.1.0.dylib)
    set(GLFW ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw3/libglfw.dylib)
    set(GLFW3 ${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw3/libglfw.3.dylib)

    # PCL, REALSENSE2, OPENCV, SOIL2*

    set(LIBRARIES ${GLEW} ${GLFW} ${GLFW3} )

endif()
include_directories(include)

add_executable(shadows ${SOURCES} include/app_state.h src/r_realsense.cpp include/r_realsense.h src/pcl_wrapper.cpp include/pcl_wrapper.h)
target_link_libraries(shadows ${LIBRARIES})









