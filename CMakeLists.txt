cmake_minimum_required(VERSION 3.15)
project(RealSense2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build")

find_library(GL libGL.so.1 HINTS /usr/lib/x86_64-linux-gnu REQUIRED)
find_library(GLEW GLEW REQUIRED)
find_library(GLFW libglfw.so.3 HINTS /usr/lib/x86_64-linux-gnu REQUIRED)
find_library(REALSENSE2 realsense2 REQUIRED)
find_library(SOIL2 soil2-debug HINTS /usr/lib)
find_library(PDAL libpdalcpp.so REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES "src/*.cpp")
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(realsense2 ${SOURCES} src/pcl_utilities.cpp include/pcl_utilities.h)
MESSAGE("STATUS ${GL} ${GLEW} ${GLFW} ${REALSENSE2} ${SOIL2} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PDAL}")

target_link_libraries(realsense2
        ${SOIL2}
        ${GLEW}
        ${GLFW}
        ${REALSENSE2}
        ${GL}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${PDAL})