#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       TestUser.py
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
#       \author/s: Gajamohan Mohanarajah 
#       
#       

import roslib; roslib.load_manifest('rceBinaryMsgDebug')

import sensor_msgs.msg
import rospy

import os.path
import Image
from StringIO import StringIO

_ENCODINGMAP_ROS_TO_PY = { 'mono8' : 'L', 'rgb8' : 'RGB', 'rgba8' : 'RGBA', 'yuv422' : 'YCbCr' }

def image_ROSRoPy(rosMsg):
    # Convert to PIL Image
        pil = Image.fromstring( _ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                                (rosMsg.width, rosMsg.height),
                                rosMsg.data,
                                'raw',
                                _ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                                0,
                                1 )

        # Save to StringIO
        img = StringIO()
        pil.save(img, 'PNG')
        return img

def cb(img):
    i = image_ROSRoPy(img)
    
    with open('test.png', 'w') as f:
       f.write(i.getvalue())
    
    print('Image received!')

if __name__ == "__main__":
    rospy.init_node('image_saver')
    s = rospy.Subscriber('circle', sensor_msgs.msg.Image, cb)
    rospy.spin()

