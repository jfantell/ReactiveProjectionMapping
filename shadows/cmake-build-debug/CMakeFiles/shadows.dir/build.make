# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/hayley/RealNVME/ComputerGraphicsProject/shadows

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/shadows.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shadows.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shadows.dir/flags.make

CMakeFiles/shadows.dir/src/camera.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/shadows.dir/src/camera.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/camera.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/camera.cpp

CMakeFiles/shadows.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/camera.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/camera.cpp > CMakeFiles/shadows.dir/src/camera.cpp.i

CMakeFiles/shadows.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/camera.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/camera.cpp -o CMakeFiles/shadows.dir/src/camera.cpp.s

CMakeFiles/shadows.dir/src/indexbuffer.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/indexbuffer.cpp.o: ../src/indexbuffer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/shadows.dir/src/indexbuffer.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/indexbuffer.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/indexbuffer.cpp

CMakeFiles/shadows.dir/src/indexbuffer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/indexbuffer.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/indexbuffer.cpp > CMakeFiles/shadows.dir/src/indexbuffer.cpp.i

CMakeFiles/shadows.dir/src/indexbuffer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/indexbuffer.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/indexbuffer.cpp -o CMakeFiles/shadows.dir/src/indexbuffer.cpp.s

CMakeFiles/shadows.dir/src/inputhandler.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/inputhandler.cpp.o: ../src/inputhandler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/shadows.dir/src/inputhandler.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/inputhandler.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/inputhandler.cpp

CMakeFiles/shadows.dir/src/inputhandler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/inputhandler.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/inputhandler.cpp > CMakeFiles/shadows.dir/src/inputhandler.cpp.i

CMakeFiles/shadows.dir/src/inputhandler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/inputhandler.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/inputhandler.cpp -o CMakeFiles/shadows.dir/src/inputhandler.cpp.s

CMakeFiles/shadows.dir/src/loadstb_image.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/loadstb_image.cpp.o: ../src/loadstb_image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/shadows.dir/src/loadstb_image.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/loadstb_image.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/loadstb_image.cpp

CMakeFiles/shadows.dir/src/loadstb_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/loadstb_image.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/loadstb_image.cpp > CMakeFiles/shadows.dir/src/loadstb_image.cpp.i

CMakeFiles/shadows.dir/src/loadstb_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/loadstb_image.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/loadstb_image.cpp -o CMakeFiles/shadows.dir/src/loadstb_image.cpp.s

CMakeFiles/shadows.dir/src/main.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/shadows.dir/src/main.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/main.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/main.cpp

CMakeFiles/shadows.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/main.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/main.cpp > CMakeFiles/shadows.dir/src/main.cpp.i

CMakeFiles/shadows.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/main.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/main.cpp -o CMakeFiles/shadows.dir/src/main.cpp.s

CMakeFiles/shadows.dir/src/mesh.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/mesh.cpp.o: ../src/mesh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/shadows.dir/src/mesh.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/mesh.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/mesh.cpp

CMakeFiles/shadows.dir/src/mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/mesh.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/mesh.cpp > CMakeFiles/shadows.dir/src/mesh.cpp.i

CMakeFiles/shadows.dir/src/mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/mesh.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/mesh.cpp -o CMakeFiles/shadows.dir/src/mesh.cpp.s

CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.o: ../src/pcl_wrapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/pcl_wrapper.cpp

CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/pcl_wrapper.cpp > CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.i

CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/pcl_wrapper.cpp -o CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.s

CMakeFiles/shadows.dir/src/r_realsense.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/r_realsense.cpp.o: ../src/r_realsense.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/shadows.dir/src/r_realsense.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/r_realsense.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/r_realsense.cpp

CMakeFiles/shadows.dir/src/r_realsense.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/r_realsense.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/r_realsense.cpp > CMakeFiles/shadows.dir/src/r_realsense.cpp.i

CMakeFiles/shadows.dir/src/r_realsense.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/r_realsense.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/r_realsense.cpp -o CMakeFiles/shadows.dir/src/r_realsense.cpp.s

CMakeFiles/shadows.dir/src/shader.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/shader.cpp.o: ../src/shader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/shadows.dir/src/shader.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/shader.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/shader.cpp

CMakeFiles/shadows.dir/src/shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/shader.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/shader.cpp > CMakeFiles/shadows.dir/src/shader.cpp.i

CMakeFiles/shadows.dir/src/shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/shader.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/shader.cpp -o CMakeFiles/shadows.dir/src/shader.cpp.s

CMakeFiles/shadows.dir/src/vertexbuffer.cpp.o: CMakeFiles/shadows.dir/flags.make
CMakeFiles/shadows.dir/src/vertexbuffer.cpp.o: ../src/vertexbuffer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/shadows.dir/src/vertexbuffer.cpp.o"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shadows.dir/src/vertexbuffer.cpp.o -c /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/vertexbuffer.cpp

CMakeFiles/shadows.dir/src/vertexbuffer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shadows.dir/src/vertexbuffer.cpp.i"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/vertexbuffer.cpp > CMakeFiles/shadows.dir/src/vertexbuffer.cpp.i

CMakeFiles/shadows.dir/src/vertexbuffer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shadows.dir/src/vertexbuffer.cpp.s"
	g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/hayley/RealNVME/ComputerGraphicsProject/shadows/src/vertexbuffer.cpp -o CMakeFiles/shadows.dir/src/vertexbuffer.cpp.s

# Object files for target shadows
shadows_OBJECTS = \
"CMakeFiles/shadows.dir/src/camera.cpp.o" \
"CMakeFiles/shadows.dir/src/indexbuffer.cpp.o" \
"CMakeFiles/shadows.dir/src/inputhandler.cpp.o" \
"CMakeFiles/shadows.dir/src/loadstb_image.cpp.o" \
"CMakeFiles/shadows.dir/src/main.cpp.o" \
"CMakeFiles/shadows.dir/src/mesh.cpp.o" \
"CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.o" \
"CMakeFiles/shadows.dir/src/r_realsense.cpp.o" \
"CMakeFiles/shadows.dir/src/shader.cpp.o" \
"CMakeFiles/shadows.dir/src/vertexbuffer.cpp.o"

# External object files for target shadows
shadows_EXTERNAL_OBJECTS =

../build/shadows: CMakeFiles/shadows.dir/src/camera.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/indexbuffer.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/inputhandler.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/loadstb_image.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/main.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/mesh.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/pcl_wrapper.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/r_realsense.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/shader.cpp.o
../build/shadows: CMakeFiles/shadows.dir/src/vertexbuffer.cpp.o
../build/shadows: CMakeFiles/shadows.dir/build.make
../build/shadows: /usr/lib/libsoil2-debug.a
../build/shadows: /usr/lib/x86_64-linux-gnu/libGLEW.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libglfw.so.3
../build/shadows: /usr/lib/x86_64-linux-gnu/librealsense2.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libGL.so.1
../build/shadows: /usr/local/lib/libopencv_gapi.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_stitching.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_aruco.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_bgsegm.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_bioinspired.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_ccalib.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_dnn_objdetect.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_dnn_superres.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_dpm.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_face.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_freetype.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_fuzzy.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_hdf.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_hfs.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_img_hash.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_line_descriptor.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_quality.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_reg.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_rgbd.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_saliency.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_stereo.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_structured_light.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_superres.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_surface_matching.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_tracking.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_videostab.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_viz.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_xfeatures2d.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_xobjdetect.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_xphoto.so.4.1.2
../build/shadows: /usr/local/lib/libpcl_surface.so
../build/shadows: /usr/local/lib/libpcl_keypoints.so
../build/shadows: /usr/local/lib/libpcl_tracking.so
../build/shadows: /usr/local/lib/libpcl_recognition.so
../build/shadows: /usr/local/lib/libpcl_stereo.so
../build/shadows: /usr/local/lib/libpcl_outofcore.so
../build/shadows: /usr/local/lib/libpcl_people.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libboost_system.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libqhull.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libfreetype.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libz.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libjpeg.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libpng.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libtiff.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libexpat.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../build/shadows: /usr/local/lib/libopencv_shape.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_highgui.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_datasets.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_plot.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_text.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_dnn.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_ml.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_phase_unwrapping.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_optflow.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_ximgproc.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_video.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_videoio.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_imgcodecs.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_objdetect.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_calib3d.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_features2d.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_flann.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_photo.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_imgproc.so.4.1.2
../build/shadows: /usr/local/lib/libopencv_core.so.4.1.2
../build/shadows: /usr/local/lib/libpcl_registration.so
../build/shadows: /usr/local/lib/libpcl_segmentation.so
../build/shadows: /usr/local/lib/libpcl_features.so
../build/shadows: /usr/local/lib/libpcl_filters.so
../build/shadows: /usr/local/lib/libpcl_sample_consensus.so
../build/shadows: /usr/local/lib/libpcl_ml.so
../build/shadows: /usr/local/lib/libpcl_visualization.so
../build/shadows: /usr/local/lib/libpcl_search.so
../build/shadows: /usr/local/lib/libpcl_kdtree.so
../build/shadows: /usr/local/lib/libpcl_io.so
../build/shadows: /usr/local/lib/libpcl_octree.so
../build/shadows: /usr/local/lib/libpcl_common.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libfreetype.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libGLEW.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1.1
../build/shadows: /usr/lib/x86_64-linux-gnu/libz.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libSM.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libICE.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libX11.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libXext.so
../build/shadows: /usr/lib/x86_64-linux-gnu/libXt.so
../build/shadows: CMakeFiles/shadows.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable ../build/shadows"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shadows.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shadows.dir/build: ../build/shadows

.PHONY : CMakeFiles/shadows.dir/build

CMakeFiles/shadows.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shadows.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shadows.dir/clean

CMakeFiles/shadows.dir/depend:
	cd /media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/hayley/RealNVME/ComputerGraphicsProject/shadows /media/hayley/RealNVME/ComputerGraphicsProject/shadows /media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug /media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug /media/hayley/RealNVME/ComputerGraphicsProject/shadows/cmake-build-debug/CMakeFiles/shadows.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shadows.dir/depend

