In order to run this executable, you must install install several shared libraries on your computer.


## Installation

In order to develop OpenGL applications on Ubuntu 18.04 follow the instructions below:

1. Open a terminal window

2. Install GLEW: `sudo apt install sudo apt install libglew-dev`

3. Install GLFW3: `sudo apt install libglfw3-dev`

4. Install RealSense2 SDK: follow (well written) instructions at `https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md`

5. Install OpenCV: `sudo apt install libopencv-dev`

6. Install GLM: `sudo apt install libglm-dev`

7. Install SOIL2:

   a. Install premake4: `sudo apt install premake4`

   b. Download SOIL2 source code from: `https://bitbucket.org/SpartanJ/soil2/downloads/`

   c. Unzip the source code files; navigate to the unzipped directory in the terminal.

   d. Run `premake4 gmake`

   e. Run `make -C make/linux` to build a static library (Ignore the error messages)

   f. Copy the static library to `/usr/lib/`: `sudo cp lib/linux/libsoil2-debug.a /usr/lib`
   
   g. Install libsdl2-dev: `sudo apt install libsdl2-dev`
   
   f. Copy the header files to `usr/include`: `sudo cp -r src/SOIL2 /usr/include`

8. Install PCL

    a. Clone git repo: `https://github.com/PointCloudLibrary/pcl.git`
    
    b. Extract repository to folder of choosing and navigate to this repository in terminal.
    
    c. Install Eigen3: `sudo apt install libeigen3-dev libflann-dev libvtk7-dev libboost-all-dev`
    
    c. Run `mkdir build && cd build`
   
9. Install PDAL: `sudo apt install libpdal-dev`
