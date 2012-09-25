#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     scanner.py
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

import os, sys

try:
    import Image
except ImportError:
    print('Requires Python Imaging Library.\n')
    exit(0)

import roslib; roslib.load_manifest('barcode')
import rospy
import sensor_msgs.msg

from barcode.srv import Scanner


_ENCODE_MAP = {'L' : 'mono8', 'RGB' : 'rgb8', 'RGBA' : 'rgba8',
               'YCbCr' : 'yuv422'}
_CHANNEL_MAP = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }


def main():
    if len(sys.argv) != 2 :
        print('Usage: scanner.py [path to image]')
        return 0
    
    path = sys.argv[1]
    
    if not os.path.isfile(path):
        print('Path is not a file.')
    
    img = Image.open(path)
    rosImg = sensor_msgs.msg.Image()
    rosImg.encoding = _ENCODE_MAP[img.mode]
    (rosImg.width, rosImg.height) = img.size
    rosImg.step = _CHANNEL_MAP[img.mode] * rosImg.width
    rosImg.data = img.tostring()
    
    rospy.wait_for_service('barcodeScanner')
    
    try:
        query = rospy.ServiceProxy('barcodeScanner', Scanner)
        response = query(rosImg)
    except rospy.ServiceException as e:
        print('Service call failed: {0}'.format(e))
        return 1
    
    if response.barcode:
        for barcode in response.barcode:
            print('{0} - {1}'.format(barcode.type, barcode.gtin))
    else:
        print('No barcodes found.')


if __name__ == '__main__':
    main()
