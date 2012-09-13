#!/bin/sh

. /opt/ros/fuerte/setup.sh
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"/opt/rce/test"
/home/dominique/rce/framework/initMachine.py $*

