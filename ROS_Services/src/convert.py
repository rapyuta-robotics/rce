#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       convert.py
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
import roslib; roslib.load_manifest('ROS_Services')
import sensor_msgs.msg
import rospy

# general imports
from datetime import datetime
import time

# Import PIL (Python Imaging Library)
try:
	import PIL.Image
except ImportError:
	print('Requires Python Imaging Library.\n')
	exit(0)

_ENCODINGMAP_PY_TO_ROS={'L' : 'mono8', 'RGB' : 'rgb8', 'RGBA' : 'rgba8', 'YCbCr' : 'yuv422'}
_ENCODINGMAP_ROS_TO_PY={'mono8' : 'L', 'rgb8' : 'RGB', 'rgba8' : 'RGBA', 'yuv422' : 'YCbCr'}
_PIL_MODE_CHANNELS = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }

def ROSToPyDict(rosObj):
	""" Converts a ROS message to a python dictionary instance. If
		the given object is not a ROS message the object is retruned
		unchanged.
	"""
	if isinstance(rosObj, rospy.Time):
		return time_ROSToPy(rosObj)
	
	if isinstance(rosObj, sensor_msgs.msg.Image):
		return image_ROSToPy(rosObj)
	
	if not isinstance(rosObj, roslib.message.Message):
		if not isinstance(rosObj, list):
			return rosObj
		
		return map(ROSToPyDict, rosObj)
	
	input={}
	for key in rosObj.__slots__:
		input[key]=ROSToPyDict(getattr(rosObj, key))
	
	return input

def time_PyToROS(timeObj):
	""" Converts a python datetime.datetime instance to a ROS Time instance.
		If the given object is not a datetime.datetime instance the
		object is returned unchanged.
	"""
	if isinstance(timeObj, datetime):
		return rospy.Time.from_sec(time.mktime(timeObj.timetuple()))
	else:
		return timeObj

def time_ROSToPy(timeObj):
	""" Converts a ROS Time instance to a python datetime.datetime instance.
		If the given object is not a ROS message the object is returned
		unchanged.
	"""
	if isinstance(timeObj, rospy.Time):
		return datetime.fromtimestamp(timeObj.to_sec())
	else:
		return timeObj

def image_PyToROS(imageObj):
	""" Converts a python PIL.Image instance to a ROS sensor.msgs.Image
		instance. If the given object is not a datetime.datetime
		instance the object is returned unchanged.
	"""
	if isinstance(imageObj, PIL.Image.Image):
		rosimage=sensor_msgs.msg.Image()
		rosimage.encoding=_ENCODINGMAP_PY_TO_ROS[imageObj.mode]
		(rosimage.width, rosimage.height)=imageObj.size
		rosimage.step=_PIL_MODE_CHANNELS[imageObj.mode]*rosimage.width
		rosimage.data = imageObj.tostring()
		return rosimage
	else:
		return imageObj

def image_ROSToPy(imageObj):
	""" Converts a ROS sensor.msgs.Image instance to a python PIL.Image
		instance. If the given object is not a datetime.datetime
		instance the object is returned unchanged.
	"""
	if isinstance(imageObj, sensor_msgs.msg.Image):
		return PIL.Image.fromstring(_ENCODINGMAP_ROS_TO_PY[imageObj.encoding],
									(imageObj.width, imageObj.height),
									imageObj.data,
									'raw',
									_ENCODINGMAP_ROS_TO_PY[imageObj.encoding],
									0,
									1)
	else:
		return imageObj
