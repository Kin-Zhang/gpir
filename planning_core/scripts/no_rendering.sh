#!/bin/bash

source /opt/ros/melodic/setup.sh
source ~/research/gp_ws/devel/setup.sh
roscd planning_core/simulation/carla/scripts
pwd
./config.py --no-rendering