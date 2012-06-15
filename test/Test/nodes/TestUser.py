#!/usr/bin/env python
import roslib; roslib.load_manifest('Test')

from Test.srv import *
import sensor_msgs.msg
import rospy

import os.path
import Image
from StringIO import StringIO

_ENCODINGMAP_ROS_TO_PY = { 'mono8' : 'L', 'rgb8' : 'RGB', 'rgba8' : 'RGBA', 'yuv422' : 'YCbCr' }

def image_ROSRoPy(rosMsg):
    # Convert to PIL Image
        pil = Image.fromstring( _ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                                (rosMsg.width, rosMsg.height),
                                rosMsg.data,
                                'raw',
                                _ENCODINGMAP_ROS_TO_PY[rosMsg.encoding],
                                0,
                                1 )

        # Save to StringIO
        img = StringIO()
        pil.save(img, 'PNG')
        return img

def main():
    s = rospy.ServiceProxy('test', QueryTest)
    r = s(4, 7)
    i = image_ROSRoPy(r.img)
    print 'Sum: {0}'.format(r.sum)
    
    with open('test.png', 'w') as f:
       f.write(i.getvalue())

if __name__ == "__main__":
    main()
