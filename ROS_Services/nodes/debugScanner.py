#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       debugScanner.py
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


from __future__ import print_function

# ROS specific imports
import roslib; roslib.load_manifest('ROS_Services')
from ROS_Services.srv import *
import rospy

from sensor_msgs.msg import Image
import convert

# main specific imports
import os, sys

# Import PIL (Python Imaging Library)
try:
	import PIL.Image
except ImportError:
	print('Requires Python Imaging Library.\n')
	exit(0)

def getImage(path):
	if os.path.isfile(path):
		return PIL.Image.open(path)
	else:
		return None

def run(img):
	rospy.wait_for_service('Scanner')
	try:
		query = rospy.ServiceProxy('Scanner', QueryScanner)
		response = query(convert.image_PyToROS(img))
		return response.gtin
	except rospy.ServiceException as e:
		print('Service call failed: {0}'.format(e))

def main():
	if len(sys.argv) != 2 :
		print('Usage: debugScanner.py [path to image]')
		return 0
	
	print('run...')
	print(run(getImage(sys.argv[1])))

if __name__ == '__main__':
	main()
