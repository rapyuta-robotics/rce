#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     paramTest.py
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

import sys

import roslib; roslib.load_manifest('Test')
import rospy

from Test.srv import ParameterTest


def callback(arg):
    msg = 'int:   {0}\nstr:   {1}\nfloat: {2}\nbool:  {3}'  #\narray: {4}'
    return msg.format(rospy.get_param('int'), rospy.get_param('str'),
                      rospy.get_param('float'), rospy.get_param('bool'))  #,
                      #rospy.get_param('array'))


def parameter_test_server(arg):
    rospy.init_node('parameter_test_server')
    rospy.Service('parameterTest', ParameterTest, lambda req: callback(arg))
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: paramTest.py [arg]')
        exit(1)

    parameter_test_server(sys.argv[1])
