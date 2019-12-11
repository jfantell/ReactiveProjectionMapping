project(SHADOWS)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build")
add_definitions(-D_DEBUG)



if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()

if(LINUX)

    file(GLOB SOURCES "src/*.cpp")
    set(NEW_SOURCES src/camera.cpp src/indexbuffer.cpp src/inputhandler.cpp src/loadstb_image.cpp src/main.cpp src/mesh.cpp src/pcl_wrapper.cpp src/r_floor.cpp
            src/r_lightmarker.cpp src/r_objmodel.cpp src/r_abbit.cpp src/r_realsense.cpp src/scene.cpp src/shader.cpp src/vertexbuffer.cpp)
    include_directories(include)

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
    add_executable(shadows ${NEW_SOURCES})
    target_link_libraries(shadows ${LIBRARIES})

else()
    message("MAC OS")
    set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -g3 -Wno-deprecated")
    set(GCC_COVERAGE_LINK_FLAGS    "-Iinclude -framework OpenGL")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build")

#    file(GLOB SOURCES "src/*.cpp")
#    message("SOURCES: ${SOURCES}")
    set(NEW_SOURCES src/camera.cpp src/indexbuffer.cpp src/inputhandler.cpp src/loadstb_image.cpp src/main.cpp src/mesh.cpp src/pcl_wrapper.cpp src/r_realsense.cpp src/shader.cpp src/vertexbuffer.cpp)
    include_directories(include)

    # Listen here, kid. FINDGlew.cmake, the module that's supposed to do all this for pcl
    # is seriously broken.  I tried all the solutions on the github issues, like this one
    # https://github.com/PointCloudLibrary/pcl/issues/1133
    # so instead of wasting my time, I whipped up a nasty little hack. just make sure the following is accurate.
    set(GLEW_FOUND 1)
    set(GLEW_INCLUDE_DIR /usr/local/Cellar/glew/2.1.0/include)
    set(GLEW_INCLUDE_DIRS /usr/local/Cellar/glew/2.1.0/include)
    set(GLEW_LIBRARIES /usr/local/Cellar/glew/2.1.0/lib/libGLEW.2.1.0.dylib /usr/local/Cellar/glew/2.1.0/lib/libGLEW.2.1.dylib)

    # include glfw
    set(GLFW_FOUND 1)
    set(GLFW_INCLUDE_DIR /usr/local/Cellar/glfw/3.3/include)
    set(GLFW_INCLUDE_DIRS /usr/local/Cellar/glfw/3.3/include)
    set(GLFW_LIBRARIES /usr/local/Cellar/glfw/3.3/lib/libglfw.dylib /usr/local/Cellar/glfw/3.3/lib/libglfw.3.dylib)


    find_library(GLEW libGLEW.2.1.0.dylib REQUIRED)
    find_library(GLFW libglfw.dylib REQUIRED)
    find_library(REALSENSE2 librealsense2.dylib REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(PCL 1.3 REQUIRED)

    # Per PCL documentation
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})

    include_directories(/usr/local/Cellar/eigen/3.3.7/include/eigen3)

    if(GLEW_FOUND)
        include_directories(${GLEW_INCLUDE_DIRS})
        message("GLEW LIBRARIES: ${GLEW_LIBRARIES}")
    endif()

    if(PCL_FOUND)
        include_directories(${PCL_INCLUDE_DIRS})
        message("PCL LIBRARIES: ${PCL_LIBRARIES}")
    endif()

     if(GLFW_FOUND)
         include_directories(${GLFW_INCLUDE_DIRS})
         message("GLFW LIBRARIES: ${GLFW_LIBRARIES}")
     endif()

    if(REALSENSE2_FOUND)
        message("hey")
    endif()
    # PCL, REALSENSE2, OPENCV, SOIL2

    set(LIBRARIES ${GLEW_LIBRARY}
            ${GLFW_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OPENCV}
            ${REALSENSE2})
    add_executable(shadows ${NEW_SOURCES})
    target_link_libraries(shadows ${LIBRARIES})

endif()









