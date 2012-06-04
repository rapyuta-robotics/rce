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

    #test topics
    msg = {"linear":{"x":1.4,"y":0,"z":0},"angular":{"x":2,"y":0,"z":4}};
    typeStr = "geometry_msgs/Twist"
    ROSmsg = proxy.specify(typeStr,msg)
    print ROSmsg

    #test service
    requestMsg = {"currentPose":{"position":{"x":0,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}};
    cls = proxy.srvClassFromTypeString('re_srvs/getNextPose')
    ROSsrvrequest = proxy.specify(cls._request_class,requestMsg)
    print ROSsrvrequest
    
    