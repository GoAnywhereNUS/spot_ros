#!/bin/bash

source /opt/ros/noetic/setup.bash
source /data/test_spot_arm/wrkspc/devel/setup.bash

# Activate environment
source /data/test_spot_arm/test_spot_env/bin/activate

python --version

export SPOT_ARM=1
roslaunch spot_driver buttercup.launch
