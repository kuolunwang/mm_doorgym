#! /bin/bash

apt update
apt install -y ros-melodic-dynamixel-sdk \
            ros-melodic-moveit-*
apt-get install -y ros-melodic-smach ros-melodic-smach-ros ros-melodic-executive-smach ros-melodic-smach-viewer