# Reactive Projection Mapping 
### Created By: Hayley Roy Gill, John Fantell, Jacob Thomas

A description of the project and associated documentation can be found [here](https://docs.google.com/document/d/1wBSYk6mY-V3U5HuCtDWflDMBsRbJjLrnCX-JDJTwIB0/edit?usp=sharing).

A video demonstration of the project can be found [here](https://youtu.be/G3er5F2yGaY).

This codebase was developed and tested with the [Intel RealSense D435 camera](https://www.intelrealsense.com/depth-camera-d435/). You must have an Intel RealSense camera to run this codebase.

## Usage

1. Plug an Intel RealSense camera into your computer via USB-C.
2. Compile the source code (or use the executable already included in the repo, which can be found in the `build` folder)
3. Launch the program with one the following command line arguments:
    - `1` # this will enable color projection mapping
    - `2 /path/to/image_file/` # this will enable texture projection mapping
4. Press the `f` key on the keyboard to re-scan the scene
5. Press the `r` key on the keyboard to reset the OpenGL view:
    - You can rotate the view by dragging the contents of the OpenGL window with your mouse/trackpad.   
    - You can translate the view by using the arrow keys on your keyboard.

## Ubuntu Linux Installation (Instructions tested on Ubuntu 18.04.3)

In order to run this code, please follow the instructions below.

1. Open a terminal window
2. Install OpenCV: `sudo apt install libopencv-dev`
3. Install PCL
    1. Install PCL dependencies: `sudo apt install libeigen3-dev libflann-dev libvtk7-dev libboost-all-dev`
    2. Clone git repo: https://github.com/PointCloudLibrary/pcl.git
    3. Extract repository to folder of choosing and navigate to this repository in terminal.
    4. Run `mkdir build && cd build`
    5. Run `cmake ..`
    6. Run `make`
    7. Run `sudo make install`
    
## Mac OS Installation (Instructions tested on macOS Catalina 10.15)

1. Open a terminal window
2. Install OpenCV: `brew install opencv`
3. Install PCL: `brew install pcl`

All of the other required libraries and their respective header files have been included in the project repository.
