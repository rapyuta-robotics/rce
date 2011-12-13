#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       debugSemantic.py
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

# main specific imports
import os, sys
import pickle

def run(data):
	rospy.wait_for_service('Semantic')
	try:
		query = rospy.ServiceProxy('Semantic', QuerySemantic)
		response = query(data)
		return response.synset
	except rospy.ServiceException as e:
		print('Service call failed: {0}'.format(e))

def main():
	if not os.path.isfile('dump.txt'):
		print('Run first debugWebDB with a barcode which returns at least one hit.')
		return
	
	with open('dump.txt') as f:
		data=pickle.load(f)
	
	print('run...')
	print(run(data))
	
if __name__ == '__main__':
	main()
