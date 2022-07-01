#! /bin/bash
source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/pokingbot/catkin_ws/src/real_to_sim_env/model
