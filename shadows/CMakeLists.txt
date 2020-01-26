project(SHADOWS)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build")

file(GLOB SOURCES "src/*.cpp")
include_directories(include)

if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()


if(LINUX)
    message("LINUX")

    # OpenGL
    set(OPENGL_FOUND 1)
    set(OPENGL_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/ubuntu/libGL.so.1.0.0)

    if(OPENGL_FOUND)
        message("OPENGL LIBRARIES: ${OPENGL_LIBRARIES}")
    endif()

    # GLEW
    set(GLEW_FOUND 1)
    set(GLEW_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/ubuntu/GL/)
    set(GLEW_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/ubuntu/libGLEW.so)

    if(GLEW_FOUND)
        include_directories(${GLEW_INCLUDE_DIRS})
        message("GLEW LIBRARIES: ${GLEW_LIBRARIES}")
    endif()

    # GLFW
    set(GLFW_FOUND 1)
    set(GLFW_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/ubuntu/GLFW/)
    set(GLFW_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/ubuntu/libglfw.so.3.2)

    if(GLFW_FOUND)
        include_directories(${GLFW_INCLUDE_DIRS})
        message("GLFW LIBRARIES: ${GLFW_LIBRARIES}")
    endif()

    # GLM
    set(GLM_FOUND 1)
    set(GLM_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/ubuntu/glm/)

    if(GLM_FOUND)
        include_directories(${GLM_INCLUDE_DIRS})
        message("GLM INCLUDE_DIRS: ${GLM_INCLUDE_DIRS}")
    endif()

    # REALSENSE2
    set(REALSENSE2_FOUND 1)
    set(REALSENSE2_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/ubuntu/librealsense2)
    set(REALSENSE2_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/ubuntu/librealsense2.so.2.31.0)

    if(REALSENSE2_FOUND)
        include_directories(${REALSENSE2_INCLUDE_DIRS})
        message("REALSENSE2 LIBRARIES: ${REALSENSE2_LIBRARIES}")
    endif()

    # OpenCV
    find_package(OpenCV REQUIRED)

    # PCL
    find_package(PCL 1 REQUIRED)
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})

    set(LIBRARIES
            ${OPENGL_LIBRARIES}
            ${GLEW_LIBRARIES}
            ${GLFW_LIBRARIES}
            ${REALSENSE2_LIBRARIES}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES})

else()
    # The header files and libraries for GLEW, GLFW, and LIBREALSENSE2 are included in this repo.
    # The only thing you should have to install prior to running this code
    # is OpenCV 4+ and PCL
    # Both of which can be installed via homebrew package manager
    message("MAC OS")

    # Required for OpenGL
    set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -g3 -Wno-deprecated")
    set(GCC_COVERAGE_LINK_FLAGS    "-Iinclude -framework OpenGL")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

    set(GLEW_FOUND 1)
    set(GLEW_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/macosx/glew/)
    set(GLEW_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/macosx/glew/libGLEW.dylib)

    if(GLEW_FOUND)
        include_directories(${GLEW_INCLUDE_DIRS})
        message("GLEW LIBRARIES: ${GLEW_LIBRARIES}")
    endif()

    set(GLFW_FOUND 1)
    set(GLFW_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/macosx/glfw/)
    set(GLFW_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/macosx/glfw3/libglfw.dylib)

    if(GLFW_FOUND)
        include_directories(${GLFW_INCLUDE_DIRS})
        message("GLFW LIBRARIES: ${GLFW_LIBRARIES}")
    endif()

    set(REALSENSE2_FOUND 1)
    set(REALSENSE2_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/macosx/librealsense2)
    set(REALSENSE2_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/macosx/librealsense2/librealsense2.dylib)

    if(REALSENSE2_FOUND)
        include_directories(${REALSENSE2_INCLUDE_DIRS})
        message("REALSENSE2 LIBRARIES ${REALSENSE2_LIBRARIES}")
    endif()

    find_package(OpenCV REQUIRED)
    find_package(PCL REQUIRED)

    # Per PCL documentation
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})

    if(PCL_FOUND)
        include_directories(${PCL_INCLUDE_DIRS})
        message("PCL LIBRARIES: ${PCL_LIBRARIES}")
    endif()

    set(LIBRARIES ${GLEW_LIBRARIES}
            ${GLFW_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OPENCV}
            ${REALSENSE2_LIBRARIES})

endif()

add_executable(shadows ${SOURCES})
target_link_libraries(shadows ${LIBRARIES})








