#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ImageConverter.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# ROS specific imports
import roslib; roslib.load_manifest('Administration')
import sensor_msgs.msg

# Python specific imports
import cStringIO
import PIL.Image

# Custom imports
from ConverterUtility import buildReference, resolveReference

class ImageConverter(object):
    """ Convert images from Django style to ROS style and back.
    """
    MESSAGE_TYPE = 'sensor_msgs/Image'
    
    _ENCODINGMAP_PY_TO_ROS = { 'L' : 'mono8', 'RGB' : 'rgb8', 'RGBA' : 'rgba8', 'YCbCr' : 'yuv422' }
    _ENCODINGMAP_ROS_TO_PY = { 'mono8' : 'L', 'rgb8' : 'RGB', 'rgba8' : 'RGBA', 'yuv422' : 'YCbCr' }
    _PIL_MODE_CHANNELS = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }
    
    def decode(self, rosMsgType, data, files):
        """ Convert a image stored (PIL library readable image file format)
            in a cStringIO.StrinO object to a ROS compatible message
            (sensor_msgs.Image).
        """
        imgObj = files[resolveReference(data)]
        
        if not isinstance(imgObj, cStringIO.OutputType):
            raise TypeError('Given object is not a cStringIO.StringO instance.')
        
        # Checking of image according to django.forms.fields.ImageField
        try:
            imgObj.reset()
            img = PIL.Image.open(imgObj)
            img.verify()
            
            imgObj.reset()
            img = PIL.Image.open(imgObj)
            img.load()
        except:
            raise ValueError('Content of given image could not be verified.')
        
        # Everything ok, convert PIL.Image to ROS and return it
        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = ImageConverter._ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = ImageConverter._PIL_MODE_CHANNELS[img.mode]*rosimage.width
        rosimage.data = img.tostring()
        return rosimage
    
    def encode(self, rosMsg, basename):
        """ Convert a ROS compatible message (sensor_msgs.Image) to a
            JPEG encoded image stored in a cStringIO.StrinO object.
        """
        if not isinstance(rosMsg, sensor_msgs.msg.Image):
            raise TypeError('Given object is not a sensor_msgs.msg.Image instance.')
        
        # Convert to PIL Image
        pil = PIL.Image.fromstring( ImageConverter._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                                    (rosMsg.width, rosMsg.height),
                                    rosMsg.data,
                                    'raw',
                                    ImageConverter._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                                    0,
                                    1)
        
        # Save to StringIO
        img = cStringIO.StringIO()
        pil.save(img, 'PNG')
        key = buildReference('img', basename)
        
        return (key, { key : img })
