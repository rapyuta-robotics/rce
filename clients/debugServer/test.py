#!/usr/bin/env python

#ROS related imports
import roslib; roslib.load_manifest('rospy')
import rospy
roslib.load_manifest('rosservice')
import roslib.rostime
import rosservice

#Python imports
from types import TypeType
import re

#Cloud Engine imports
from ROSProxy import ROSProxy 


if __name__ == "__main__":
    proxy = ROSProxy();
    
    #test
    msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}};
    typeStr = "geometry_msgs/Twist"


    ROSmsg = proxy.specify(typeStr,msg)
    print ROSmsg
    
    
    
    