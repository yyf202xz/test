#!/usr/bin/env python
## coding: UTF-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# 0 <= h <= 179 (色相)　OpenCVではmax=179なのでR:0(180),G:60,B:120となる
# 0 <= s <= 255 (彩度)　黒や白の値が抽出されるときはこの閾値を大きくする
# 0 <= v <= 255 (明度)　これが大きいと明るく，小さいと暗い
# 色を抽出する
LOW_COLOR = np.array([30, 64, 0])
HIGH_COLOR = np.array([90,255,255])

firstframe = 1

class ColorExtract(object):
    def __init__(self):
            self._image_sub = rospy.Subscriber('/image_raw', Image, self.callback)
            self._bridge = CvBridge()

    def callback(self, data):
        global firstframe
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
            print(type(cv_image))
        except CvBridgeError, e:
            print e
        rospy.loginfo('color_pixel=%d,err_x=%d,err_y=%d,h=%d,w=%d,c=%d')
        cv2.imshow('webcam', cv_image)
        cv2.waitKey(1)
        if firstframe == 1:
            init_rect = cv2.selectROI('webcam', cv_image, False, False)
            firstframe = 0

    
if __name__ == '__main__':
    rospy.init_node('color_extract')
    color = ColorExtract()
    #cv2.namedWindow('webcam', cv2.WND_PROP_FULLSCREEN)
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


    