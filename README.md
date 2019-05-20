# Test opencv cuda

## Installation
This package requires OpenCV and ROS as main dependencies. Please install them according to the following instructions:

* OpenCV L4T Jetson: this is a prebuild version with CUDA support that can be installed using Jetpack (4.2) (https://developer.nvidia.com/embedded/jetpack)
* ROS (kinetic): follow the instructions at http://wiki.ros.org/kinetic/Installation/Ubuntu and choose to install the ros-kinetic-ros-base version. In addition to this follow the steps to create a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in which you will need to download and the compile the following package (https://github.com/ros-perception/image_common). Make sure that during the compilation the correct version of OpenCV with CUDA support is found and linked.

Finally in order to use the Basler camera sensor on ROS the following package is needed:

* Basler ros driver: the instruction at this link (https://github.com/magazino/pylon_camera) allow to install the pylon package and set up a ROS node for the camera.

In order to download and compile the this package:
```
cd <catkin_ws_path>/src
git clone https://github.com/omichele/test_opencv_cuda.git
cd ..
catkin build (or catkin_make)
```

## Configuration of the camera
The ROS driver of the camera allows to set some parameters by modifying the file config/default.yaml. During the tests we used most often the following params:

* frame_rate: 60.0 (not reached)
* binning_x: 3
* binning_y: 2
* image_encoding: "rgb8"
* shutter_mode: "global_reset"
* exposure: 2000.0
* brightnet_continuous: true
* gain_auto: true
* exposure_auto: true
* auto_exposure_upper_limit: 10000.0

## Run the code

1. Launch the camera driver with the command:
```
roslaunch pylon_camera pylon_camera_node.launch
```

2. Run the test node giving as parameter the name of the topic where the image is published:
```
rosrun test_opencv_cuda test_opencv_cuda_node <image_raw_topic>
```
