#!/usr/bin/env python
import roslib; roslib.load_manifest('rceBinaryMsgDebug')
import rospy

import cv
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError

class imageGenerator(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("circleOut",Image)
        
        self.imgDim = (500,500)
        self.img = cv.CreateImage(self.imgDim,cv.IPL_DEPTH_8U,1)

    def callback(self,data):
        posX = self.imgDim[0]*data.x
        posY = self.imgDim[1]*data.y
        
        cv.Circle(self.img, (int(posX),int(posY)), int(self.imgDim[0]*0.1), 255, -1)
        
        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(self.img, "mono8"))
        except CvBridgeError, e:
            print e
        cv.Set(self.img,0)
        
if __name__ == '__main__':
    rospy.init_node('Pose2DSubscriber')
    ig = imageGenerator()
    rospy.Subscriber('ccPos', Pose2D, ig.callback)
    rospy.spin()
