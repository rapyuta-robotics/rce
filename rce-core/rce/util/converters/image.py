#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/util/converters/image.py
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

# Python specific imports
try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

try:
    import Image
except ImportError:
    print('Can not import Python Image Library.')
    exit(1)

# ROS specific imports
try:
    import sensor_msgs.msg
except ImportError:
    print("Can not import the required ROS Python library 'sensor_msgs'.")
    print('Make sure they are installed and the ROS Environment is setup.')
    exit(1)

# zope specific imports
from zope.interface import implements

# rce specific imports
from rce.util.converters.interfaces import ICustomROSConverter


class ImageConverter(object):
    """ Convert images from PNG file format to ROS sensor message format and
        back.
    """
    implements(ICustomROSConverter)

    MESSAGE_TYPE = 'sensor_msgs/Image'

    _ENCODINGMAP_PY_TO_ROS = { 'L' : 'mono8', 'RGB' : 'rgb8',
                               'RGBA' : 'rgba8', 'YCbCr' : 'yuv422' }
    _ENCODINGMAP_ROS_TO_PY = { 'mono8' : 'L', 'rgb8' : 'RGB',
                               'rgba8' : 'RGBA', 'yuv422' : 'YCbCr' }
    _PIL_MODE_CHANNELS = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }

    def decode(self, imgObj):
        """ Convert a image stored (PIL library readable image file format)
            in a StringIO object to a ROS compatible message
            (sensor_msgs.Image).
        """
        if not _checkIsStringIO(imgObj):
            raise TypeError('Given object is not a StringIO instance.')

        # Checking of image according to django.forms.fields.ImageField
        try:
            imgObj.seek(0)
            img = Image.open(imgObj)
            img.verify()
        except:
            raise ValueError('Content of given image could not be verified.')

        imgObj.seek(0)
        img = Image.open(imgObj)
        img.load()

        # Everything ok, convert PIL.Image to ROS and return it
        if img.mode == 'P':
            img = img.convert('RGB')

        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = ImageConverter._ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = (ImageConverter._PIL_MODE_CHANNELS[img.mode]
                         * rosimage.width)
        rosimage.data = img.tostring()
        return rosimage

    def encode(self, rosMsg):
        """ Convert a ROS compatible message (sensor_msgs.Image) to a
            PNG encoded image stored in a StringIO object.
        """
        if not isinstance(rosMsg, sensor_msgs.msg.Image):
            raise TypeError('Given object is not a sensor_msgs.msg.Image '
                            'instance.')

        # Convert to PIL Image
        pil = Image.fromstring(
                ImageConverter._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                (rosMsg.width, rosMsg.height),
                rosMsg.data,
                'raw',
                ImageConverter._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                0,
                1)

        # Save to StringIO
        img = StringIO()
        pil.save(img, 'PNG')
        return img
