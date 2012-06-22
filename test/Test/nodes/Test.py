#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Test.py
#       
#       This file is part of the RoboEarth Cloud Engine tests.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

import roslib; roslib.load_manifest('Test')

from Test.srv import *
import sensor_msgs.msg
import rospy

import os.path
import PIL.Image

_ENCODINGMAP_PY_TO_ROS = {'L' : 'mono8', 'RGB' : 'rgb8', 'RGBA' : 'rgba8', 'YCbCr' : 'yuv422'}
_PIL_MODE_CHANNELS = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }

def image_PyToROS(imageObj):
    """ Converts a python PIL.Image instance to a ROS sensor_msgs.msgs.Image
        instance. If the given object is not a datetime.datetime
        instance the object is returned unchanged.
    """
    if isinstance(imageObj, PIL.Image.Image):
        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = _ENCODINGMAP_PY_TO_ROS[imageObj.mode]
        (rosimage.width, rosimage.height) = imageObj.size
        rosimage.step = _PIL_MODE_CHANNELS[imageObj.mode] * rosimage.width
        rosimage.data = imageObj.tostring()
        return rosimage
    else:
        return imageObj

def callback(req):
    path = roslib.packages.get_pkg_dir('Test')
    img = PIL.Image.open(os.path.join(path, 'ros_org.png'))
    img = image_PyToROS(img)
    return QueryTestResponse(req.a + req.b, img)

def add_two_ints_server():
    rospy.init_node('test_server')
    s = rospy.Service('test', QueryTest, callback)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
