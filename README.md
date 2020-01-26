# RAR: Reactive Augmented Reality is a dynamic projection mapping system developed at Rensselaer Polytechnic Institute.
### Created By: Hayley Roy Gill, John Fantell, Jacob Thomas

A description of the project and associated documentation can be found [here](https://docs.google.com/document/d/1wBSYk6mY-V3U5HuCtDWflDMBsRbJjLrnCX-JDJTwIB0/edit?usp=sharing).

A video demonstration of the project can be found [here](https://youtu.be/G3er5F2yGaY).

This codebase was developed and tested with the [Intel RealSense D435 camera](https://www.intelrealsense.com/depth-camera-d435/). You must have an Intel RealSense camera to run this codebase.

## Usage

1. Plug an Intel RealSense camera into your computer via USB-C.
2. Compile the source code (or use the executable already included in the repo, which can be found in the `build` folder)
3. Launch the program with one the following command line arguments:
    a. `1` # this will enable color projection mapping
    b. `2 /path/to/image_file/` # this will enable texture projection mapping
4. Press the `f` key on the keyboard to re-scan the scene
5. Press the `r` key on the keyboard to reset the OpenGL view
    a. You can rotate the view by dragging the contents of the OpenGL window with your mouse/trackpad.
    b. You can translate the view by using the arrow keys on your keyboard.

## Ubuntu Linux Installation (Instructions tested on Ubuntu 18.04.3)

In order to run this code, please follow the instructions below.

1. Open a terminal window

2. Install GLEW: `sudo apt install libglew-dev`

3. Install GLFW3: `sudo apt install libglfw3-dev`

4. Install RealSense2 SDK: follow (well written) instructions at https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

5. Install OpenCV: `sudo apt install libopencv-dev`

6. Install GLM: `sudo apt install libglm-dev`

7. Install PCL

    a. Install PCL dependencies: `sudo apt install libeigen3-dev libflann-dev libvtk7-dev libboost-all-dev`
    
    b. Clone git repo: https://github.com/PointCloudLibrary/pcl.git

    c. Extract repository to folder of choosing and navigate to this repository in terminal.

    d. Run `mkdir build && cd build`
    
    e. Run `cmake ..`
    
    f. Run `make`
    
    g. Run `sudo make install`
