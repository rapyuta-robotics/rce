#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     converterTest.py
#
#     This file is part of the RoboEarth Cloud Engine framework.
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2012 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#     \author/s: Dominique Hunziker
#
#

import os.path

try:
    import Image
except ImportError:
    print('Requires Python Imaging Library.\n')
    exit(0)

import roslib; roslib.load_manifest('Test')
import rospy
import sensor_msgs.msg

from Test.srv import ConverterTest


_ENCODE_MAP = {'L' : 'mono8', 'RGB' : 'rgb8', 'RGBA' : 'rgba8',
               'YCbCr' : 'yuv422'}
_CHANNEL_MAP = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }


def callback(req):
    path = roslib.packages.get_pkg_dir('Test')
    img = Image.open(os.path.join(path, 'data/roboEarth_logo.png'))

    rosImg = sensor_msgs.msg.Image()
    rosImg.encoding = _ENCODE_MAP[img.mode]
    (rosImg.width, rosImg.height) = img.size
    rosImg.step = _CHANNEL_MAP[img.mode] * rosImg.width
    rosImg.data = img.tostring()

    return QueryTestResponse(req.a + req.b, rosImg)


def converter_test_server():
    rospy.init_node('test_server')
    s = rospy.Service('test', ConverterTest, callback)
    rospy.spin()


if __name__ == "__main__":
    converter_test_server()
