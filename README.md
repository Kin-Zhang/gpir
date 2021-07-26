# GPIR

## Introduction

Todo

## Prerequisites

- Carla-0.9.11
- ROS melodic/noetic
- Todo

## Build

In a catkin workspace (e.g. catkin_ws/src)

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