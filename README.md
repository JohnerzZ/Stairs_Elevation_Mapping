# Creating an elevation Map of a Staircase
# Using Intel RealSense and ROS
---
## Overview

The purpose of this project is to review software packages developed for elevation mapping with a mobile robot and more specifically with a quadruped robot. Most algorithms used to form an elevation map around a mobile robot require data regarding the position of the robot relative to its environment through time, as well as depth and color images captured in live time. The hardware we used to provide these data consists of the [Intel® RealSense™ Tracking Camera T265] and the [Intel® RealSense™ Depth Camera D435i]. The packages that we review here have been configured to accept data from these two sensors.

## Installation

### Rtabmap
1. Install [ROS noetic].
2. Install [Intel RealSense SDK 2.0].
3. Install [Intel® RealSense™ ROS] from source.
4. Install ddynamic-reconfigure and octomap-rviz-plugins:
```sh
sudo apt-get install ros-noetic-rtabmap-ros
sudo apt-get install ros-noetic-octomap-rviz-plugins
```
5. Compile by running _catkin_make_ or _catkin_build_ (whichever tool you prefer) inside your catkin workspace.
6. Run using:
```sh
roslaunch realsense2_camera rs_rtabmap.launch
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;or optionally:
```sh
roslaunch realsense2_camera rs_rtabmap.launch use_rviz:=false use_rtabmapviz:=false
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if you want to visualize using rtabmapviz instead of rviz.
7. Configure rviz
- Under Image->Image Topic choose _d400_color_image_raw_
- Check the checkbox for the _Mapcloud_ topic
- If you wish, check the checkbox for the _ColorOccupancyGrid_ topic

### Anybotics Elevation-Mapping
1. Install [ROS noetic].
2. Install [Intel RealSense SDK 2.0].
3. Install [Intel® RealSense™ ROS] from source.
4. Visit the official [Anybotics/Elevation-Mapping] repository and install all the dependencies
5. Run:
```sh
cd catkin_workspace/src
git clone https://github.com/anybotics/elevation_mapping.git
cd ../
catkin_make
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; If you wish to compile with catkin_build instead of catkin_make then instead of running catkin_make run:
```sh
sudo apt install python3-catkin-tools python3-osrf-pycommon
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
6. Ensure package is installed correctly by running the tests:
```sh
catkin_make run_tests
rostest elevation_mapping elevation_mapping.test -t
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; If you wish to compile with catkin_build then run these instead:
```sh
roscd elevation_mapping
catkin build --catkin-make-args run_tests -- --this
rostest elevation_mapping elevation_mapping.test -t
```
##### Test in simulation

7. Install Turtlebot3 Simulator:
```sh
sudo apt install ros-noetic-turtlebot3*
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;If this does not work then install the required packages manually:
```sh
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-bringup
sudo apt install ros-noetic-turtlebot3-bringup-dbgsym
sudo apt install ros-noetic-turtlebot3-description
sudo apt install ros-noetic-turtlebot3-example
sudo apt install ros-noetic-turtlebot3-fake
sudo apt install ros-noetic-turtlebot3-fake-dbgsym
sudo apt install ros-noetic-turtlebot3-gazebo
sudo apt install ros-noetic-turtlebot3-gazebo-dbgsym
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3-navigation
sudo apt install ros-noetic-turtlebot3-simulations
sudo apt install ros-noetic-turtlebot3-slam
sudo apt install ros-noetic-turtlebot3-slam-dbgsym
sudo apt install ros-noetic-turtlebot3-teleop
```

8. At: /catkin_ws/src/elevation_mapping/elevation_mapping_demos/launch:
At line 14: set gui to true.
9. Run using:
```sh
roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch
```
10. Control the turtlebot by opening a new console and running:
```sh
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
##### Test with Realsense D435i and T265
11. Copy _my.yaml_ into _/catkin_ws/src/elevation_mapping/elevation_mapping_demos/config/robots_.
12. Copy _d435i.launch_ into _/catkin_ws/src/elevation_mapping/elevation_mapping_demos/launch_.
13. Go to folder _catkin_ws/src/realsense-ros/realsense2_camera_.
and replace _rs_d400_and_t265.launch_ with the _rs_d400_and_t265.launch_ that is provided in this repository.
14. Run with:
```sh
roslaunch elevation_mapping_demos d435i.launch
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;If an error regarding rospkg occurs, try running:
```sh
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
```
15. Configure Rviz:
* As a fixed frame: choose _t265_odom_frame_
* Add topic -> By topic -> GridMap -> _elevation_map_raw_
* Add topic -> By display type -> Rviz -> _TF_

### Gradslam
1. Install [cuda 11.4].
2. Install [pytorch] from source.
To do that, first install [anaconda] if it is not already installed.
Then in a conda environment or the base environment run:
```sh
conda install astunparse numpy ninja pyyaml mkl mkl-include setuptools cmake cffi typing_extensions future six requests dataclasses
conda install -c pytorch magma-cuda113
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Note that magma-cuda113 has been made to work with cuda11.3 but also works with cuda11.4 . Because magma-cuda114 is not out yet.
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Then run:
```sh
git clone --recursive https://github.com/pytorch/pytorch
cd pytorch
export CMAKE_PREFIX_PATH=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
python setup.py install
```
3. Install [Gradslam]:
```sh
git clone https://github.com/krrish94/chamferdist.git
cd chamferdist
pip3 install .
cd ..
git clone https://github.com/gradslam/gradslam.git
cd gradslam
pip3 install -e .[dev]
```
4. Install glob2:
```sh
pip3 install glob2
```
5. Install pyrealsense2:
```sh
pip3 install pyrealsense2
```
6. Download Gradslam_Realsense.ipynb and run it with the realsense cameras plugged in.

The exact branches of the open source code used in this project can be found inside the src folder of this repository. 

   [Intel® RealSense™ Tracking Camera T265]: <https://www.intelrealsense.com/tracking-camera-t265/>
   [Intel® RealSense™ Depth Camera D435i]: <https://www.intelrealsense.com/depth-camera-d435i/>
   [Intel RealSense SDK 2.0]: <https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md>
   [ROS noetic]: <http://wiki.ros.org/noetic/Installation/Ubuntu>
   [Intel® RealSense™ ROS]: <https://github.com/IntelRealSense/realsense-ros>
   [Anybotics/Elevation-Mapping]: <https://github.com/ANYbotics/elevation_mapping>
   [cuda 11.4]: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
   [pytorch]: <https://github.com/pytorch/pytorch#from-source>
   [anaconda]: <https://www.anaconda.com/products/individual-d#download-section>
   [Gradslam]: <https://github.com/gradslam/gradslam#installation>