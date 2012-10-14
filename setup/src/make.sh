#!/bin/sh

. /opt/rce/setup.sh
ROS_LOG_DIR=/home/ros
rosmake $*
