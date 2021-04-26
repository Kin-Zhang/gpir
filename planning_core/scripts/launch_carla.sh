#!/bin/bash
# launch carla engine
gnome-terminal -- ~/carla/./CarlaUE4.sh

sleep 8s

# launch carla rosbridge
source ~/miniconda3/etc/profile.d/conda.sh
conda activate carla_py2
roslaunch planning_core carla_setup.launch