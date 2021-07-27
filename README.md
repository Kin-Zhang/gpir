# GPIR

## Introduction

<mark>TODO</mark>

## Prerequisites

- [Carla-0.9.11](https://github.com/carla-simulator/carla/releases/tag/0.9.11)
  Note: Please remember install carla.egg into your python `pip install -e *`
- ROS melodic/noetic
- [Carla-ros-bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/)
- Python env: 2.7 since we need to use ros-bridge
  Python requirement: pls see Carla/PythonAPI folder
  Note: when install opencv for python 2.7 `pip install opencv-python==4.2.0.32` [Error may occur](https://stackoverflow.com/questions/63346648/python-2-7-installing-opencv-via-pip-virtual-environment)

### Dependence
- CMake 3.13.0 or higher is required [HOW TO UPGRADE](https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu)
- glog [origin github](https://github.com/google/glog.git) / [mirror from china](https://codechina.csdn.net/mirrors/google/glog.git)
- osqp [Install doc](https://osqp.org/docs/get_started/sources.html#build-the-binaries)
- spdlog [origin github](https://github.com/gabime/spdlog)
   Note: Please remember open CMakeLists.txt after clone and add `set(CMAKE_CXX_FLAGS "-fPIC") ` 
- adolc: `sudo apt install libadolc-dev`
- Ipopt: [Install doc](https://coin-or.github.io/Ipopt/INSTALL.html)
   Note about: [HSL to IPOPT](https://stackoverflow.com/questions/58305144/trying-to-compile-hsl-to-get-ipopt)

## Build

In a **catkin workspace** (e.g. catkin_ws/src)

```bash
git clone --recursive https://github.com/jchengai/gpir
cd gpir && ./setup.sh
cd ../../ && catkin_make -DCMAKE_BUILD_TYPE=RELEASE -j4
source devel/setup.sh
```

## Run 

- start carla-0.9.11 
    ```
    cd to_your_carla_folder && ./carla.sh
    ```
    
- start carla ros bridge 
    
    ```
    roslaunch planning_core carla_setup.launch
    ```
    
- start planner 
    ```
    roslaunch planning_core planning.launch
    ```
    
- use 2D Nav Goal in Rviz to give a arbitrary goal

### Change the map

Modify `town parameter` in `carla_setup.launch` and `planning.launch`

<mark>TODO</mark>: This need change the spawn point (or use random spawn point)

## Citation

<mark>TODO</mark>