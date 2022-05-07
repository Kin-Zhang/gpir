# GPIR

## Introduction

<mark>TODO</mark>

## Prerequisites

- [Carla-0.9.11](https://github.com/carla-simulator/carla/releases/tag/0.9.11)
  Note: Please remember install carla.egg into your python `pip install -e *`

  ```bash
  cd ~/CARLA_0.9.11/PythonAPI/carla/dist/
  unzip carla-0.9.11-py2.7-linux-x86_64.egg -d carla-0.9.11-py2.7-linux-x86_64
  cd carla-0.9.11-py2.7-linux-x86_64
  gedit setup.py
  ```

  copy this to your setup.py

  ```bash
  from distutils.core import setup
  setup(name='carla',
        version='0.9.11', 
        py_modules=['carla'],
        )
  ```

  Finally:

  ```bash
  pip install -e ~/CARLA_0.9.11/PythonAPI/carla/dist/carla-0.9.11-py2.7-linux-x86_64
  ```

- ROS melodic/noetic

- Python env: 2.7 since we need to use ros-bridge
  Python requirement: pls see Carla/PythonAPI folder
  Note: when install opencv for python 2.7 `pip install opencv-python==4.2.0.32` [Error may occur](https://stackoverflow.com/questions/63346648/python-2-7-installing-opencv-via-pip-virtual-environment)

### Dependence
- CMake 3.13.0 or higher is required [HOW TO UPGRADE](https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu)

   ```bash
   # 1. Check your current version with:
   cmake --version
   # 2. Visit https://cmake.org/download/ and download the latest bash script.
   TODO 
   ```

- glog [origin github](https://github.com/google/glog.git) / [mirror from china](https://codechina.csdn.net/mirrors/google/glog.git)

   ```bash
   git clone https://github.com/google/glog.git
   cd glog
   cmake -S . -B build -G "Unix Makefiles"
   cmake --build build
   ```

   Optional: 1. for test, 2. Install the built files

   ```bash
   cmake --build build --target test
   sudo cmake --build build --target install
   ```

- osqp [Install doc](https://osqp.org/docs/get_started/sources.html#build-the-binaries)

   ```bash
   git clone --recursive https://github.com/osqp/osqp
   cd osqp
   mkdir build
   cd build
   cmake -G "Unix Makefiles" ..
   cmake --build .
   sudo cmake --build . --target install
   ```

- spdlog [origin github](https://github.com/gabime/spdlog)
   Note: ==Please remember open CMakeLists.txt after clone== and add `set(CMAKE_CXX_FLAGS "-fPIC") ` 

   ```bash
   git clone https://github.com/gabime/spdlog.git
   cd spdlog && mkdir build && cd build
   cmake .. && make -j
   sudo make install
   ```

- adolc: 

   ```bash
   sudo apt install libadolc-dev
   ```

- Ipopt: [Install doc](https://coin-or.github.io/Ipopt/INSTALL.html)
   **Note about: [HSL to IPOPT](https://stackoverflow.com/questions/58305144/trying-to-compile-hsl-to-get-ipopt) ** Must copy the `coinhsl` folder to ThirdParty-HSL

   **<u>System Dependence</u>**:

   ```bash
   sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
   ```

   If you have problems with install `gfortran`, this ./configure will not success so try to satisfy their dependence. So, you can try install by `sudo aptitude install gfortran`

   <u>**ThirdParty**</u>:

   ```bash
   git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
   cd ThirdParty-ASL
   ./get.ASL
   ./configure
   make
   sudo make install
   ```
	==Please copy `coinhsl` which is a license folder== see more on note about HSL to IPOPT
   
   ```bash
   git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
   
   cd ThirdParty-HSL
   ./configure
   make
   sudo make install
   ```
   
   ```bash
   git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
   cd ThirdParty-Mumps
   ./get.Mumps
   ./configure
   make
   sudo make install
   ```
   
   Finally Install **<u>IPOPT</u>**:
   
   ```bash
   git clone https://github.com/coin-or/Ipopt.git
   cd Ipopt
   mkdir build
   cd build
   ../configure
   make
   make test
   sudo make install
   ```


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
    cd CARLA_0.9.11 && ./carla.sh
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

OPTIONAL:

Add other agents and obstacle in the lane

- python run the keyboard.py file

  ```bash
  python keyboard.py
  ```

  W/S: increase/decrease reference speed

  A/D: change to left/right lane

  I/K: add car agent in the front/back of ego

  U/J: add car agent in the front/back left of ego

  O/L: add car agent in the front/back right of ego

  1/2/3/4: add static obstacles (only visible in rviz)

  b: trigger all other agent's autopilot

### Change the map

Modify `town parameter` in `carla_setup.launch` and `planning.launch`

<mark>**<u>TODO</u>**</mark>: This need change the spawn point (or use random spawn point)

## Citation

**<u><mark>TODO</mark></u>**