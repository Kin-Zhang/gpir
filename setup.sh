#!/bin/bash

ROOT_DIR=$(cd $(dirname "$0"); pwd)

# extract 3D model
cd ${ROOT_DIR}/planning_core/3d_model && pwd && tar -xvf model3.tar.xz

# build gtsam
cd ${ROOT_DIR}/gp_planner/thirdparty/gtsam-4.1rc
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install"
make -j6 && make install

sudo apt install -y libpugixml-dev \ 
                    ros-${ROS_DISTRO}-ackermann-msgs \
                    ros-${ROS_DISTRO}-derived-object-msgs \ 
                    ros-${ROS_DISTRO}-jsk-recognition-msgs \
                    ros-${ROS_DISTRO}-jsk-rviz-plugins 
