#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     zbarScanner.py
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
#     Note:   This barcode scanner uses the 'zbar bar code reader' library
#             and its python wrapper, which are available under the GNU
#             Lesser General Public License, version 2.1.
#             The sourcecode is available under:
#                 http://zbar.sourceforge.net/
#

try:
    import Image
except ImportError:
    print('Requires Python Imaging Library.\n')
    exit(0)

import roslib; roslib.load_manifest('barcode')
import rospy

# Import zbar (barcode reader library)
try:
    import zbar
except ImportError:
    print('Requires zbar.\n')
    exit(0)

from barcode.msg import Barcode
from barcode.srv import Scanner


_MAP = { 'mono8' : 'L', 'rgb8' : 'RGB', 'rgba8' : 'RGBA', 'yuv422' : 'YCbCr' }


def scan(req):
    pil = Image.fromstring(_MAP[req.image.encoding],
                           (req.image.width, req.image.height),
                           req.image.data, 'raw', _MAP[req.image.encoding],
                           0, 1)
    pil = pil.convert('L')
    width, height = pil.size
    raw = pil.tostring()
    
    image = zbar.Image(width, height, 'Y800', raw)
    
    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')
    scanner.scan(image)
    
    barcodes = []
    
    for symbol in image:
        rospy.loginfo('Found: {0} - {1}'.format(symbol.type, symbol.data))
        barcodes.append(Barcode(symbol.data, str(symbol.type)))
    
    return (barcodes,)


def scanner_server():
    rospy.init_node('scannerNode')
    rospy.Service('barcodeScanner', Scanner, scan)
    rospy.spin()


if __name__ == '__main__':
    scanner_server()
