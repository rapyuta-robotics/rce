#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       debugWebDB.py
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
import roslib; roslib.load_manifest('BarCodeService')
from BarCodeService.srv import *
import rospy

# main specific imports
import sys
import pickle

def run(gtin):
    rospy.wait_for_service('WebDB')
    try:
        query = rospy.ServiceProxy('WebDB', QueryWebDB)
        response = query(gtin)
        return response.data
    except rospy.ServiceException as e:
        print('Service call failed: {0}'.format(e))

def main():
    if len(sys.argv) != 2 or len(sys.argv[1]) != 13:
        print('Usage: debugWebDB.py [13-digit gtin]')
        return 0
    
    print('run...')
    result=run(sys.argv[1])
    
    print('Found {0} hits.'.format(len(result)))
    
    with open('dump.txt', 'w') as f:
        pickle.dump(result, f)

if __name__ == '__main__':
    main()
