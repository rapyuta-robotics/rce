#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     info.py
#     
#     This file is part of the RoboEarth Cloud Engine test.
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

import roslib; roslib.load_manifest('barcode')
import rospy
import sensor_msgs.msg

from barcode.msg import Barcode, DBInfo


def printInfo(msg):
    print(msg)


def main():
    if len(sys.argv) != 3 :
        print('Usage: webDB.py [GTIN] [type of GTIN]')
        return 0
    
    gtin = sys.argv[1]
    gtinType = sys.argv[2]
    
    rospy.init_node('debugBarcodeInfoNode')
    
    rospy.Subscriber('barcodeDBInfo', DBInfo, printInfo)
    pub = rospy.Publisher('barcodeWebDBQuery', Barcode, latch=True)
    pub.publish(gtin, gtinType)
    rospy.spin()


if __name__ == '__main__':
    main()
