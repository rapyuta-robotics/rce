#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       debug%Service%.py
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
import roslib; roslib.load_manifest('%Package%')
from %Package%.srv import Query%Service%
import rospy

def compileInput():
    ###
    ### Add your code here.
    ###
    ### This function should return a dictonary which keys correspond to
    ### the input fields of the service.
    ###
    ### Example:
    ###
    ### return {'numbers' : [1,2,3] }
    ###

def run(data):
    rospy.wait_for_service('%Service%')
    try:
        query = rospy.ServiceProxy('%Service%', Query%Service%)
        response = query(**data)
        return response.### Edit ###
    except rospy.ServiceException as e:
        print('Service call failed: {0}'.format(e))

def main():
    print('run...')
    print(run(compileInput()))

if __name__ == '__main__':
    main()
