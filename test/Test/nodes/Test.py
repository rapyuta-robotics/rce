#!/usr/bin/env python
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
