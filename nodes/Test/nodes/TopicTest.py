#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       TopicTest.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
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

import roslib; roslib.load_manifest('Test')
import rospy

from std_msgs import Int32
from Test.srv import TopicService, TopicServiceResponse

global currentSum
currentSum = 0

def addInt(req):
    global currentSum
    #print '--------------------\nadd Int\n{0} + {1} = {2}--------------------'.format(currentSum, req.data, req.data+currentSum)
    currentSum += req.data

def getSum(req):
    return TopicServiceResponse(currentSum)

def main():
    #print '--------------------\nStart Topic-Test\n--------------------'
    rospy.init_node('topic_test')
    srv = rospy.Service('getSum', TopicService, getSum)
    topic = rospy.Subscriber('addInt', Int32, addInt)
    rospy.spin()

if __name__ == '__main__':
    main()
