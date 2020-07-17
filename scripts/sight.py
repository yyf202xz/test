#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from img_recog.msg import points
from cv_bridge import CvBridge, CvBridgeError

def triangle(image, x, y):
    pt1 = (x, y)
    pt2 = (x-10, y+20)
    pt3 = (x+10, y+20)

    cv2.circle(image, pt1, 0, (0,0,255), -1)
    cv2.circle(image, pt2, 0, (0,0,255), -1)
    cv2.circle(image, pt3, 0, (0,0,255), -1)

    triangle_cnt = np.array( [pt1, pt2, pt3] )
    Image = cv2.drawContours(image, [triangle_cnt], 0, (0,0,255), -1)

    return Image

def project_n_undistort(a,world):
    b = np.dot(a,world)
    b = b/b[2]
    b = b[0:2]
    b = cv2.undistortPoints(b, mtx, dist, None, mtx)
    b = b[0][0]
    return b

class Sight(object):
    def __init__(self):
        self.sight_pub = rospy.Publisher('Sight', Image, queue_size=1)
        self.image_sub1 = rospy.Subscriber('RGB/image_raw', Image, self.callback)
        self.undistort_pub = rospy.Publisher('undistort_image', Image, queue_size=1)
        
        self.spectrum_pub = rospy.Publisher('Rotated_spectrum_image', Image, queue_size=1)
        self.image_sub2 = rospy.Subscriber('spectrum/image_raw', Image, self.rotate_callback)
        self.point_pub = rospy.Publisher('target_point', points, queue_size=1)
        self.target = points()

        self._bridge = CvBridge()

    def callback(self,data):
        try:
            ImgFrame = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e

        dimensions = ImgFrame.shape[:2]
        dimensions = dimensions[::-1]

        #歪み修正   
        ImgFrame = cv2.undistort(ImgFrame, mtx, dist, None, mtx)

        try:
            self.undistort_pub.publish(self._bridge.cv2_to_imgmsg(ImgFrame, 'bgr8'))
        except CvBridgeError, e:
            print e
       
        target = np.round((c+c2)/2)
        
        #point1:center point, point2,3:vertex
        self.target.point1_x = target[0]
        self.target.point1_y = target[1]
        self.target.point2_x = c[0]
        self.target.point2_y = c[1]
        self.target.point3_x = c2[0]
        self.target.point3_y = c2[1]

        #三角形描画
        #input distance
        triangle(ImgFrame,int(b[0]), int(b[1]))
        ImgFrame = cv2.rectangle(ImgFrame,(int(c[0]),int(c[1])),(int(c2[0]),int(c2[1])),(0,255,0),3)
        cv2.putText(ImgFrame, str(dis)+'m', (int(c[0]+10), int(b[1])), fontface, 1.0, (0,0,255), fontthick , cv2.LINE_AA)
        target = np.round((c+c2)/2)
        
        
        #>3m
        triangle(ImgFrame,int(b_3[0]), int(b_3[1]))
        cv2.putText(ImgFrame, '>3m', (int(b_3[0]-90), int(b_3[1])), fontface, 1.0, (0,0,255), fontthick , cv2.LINE_AA)

        try:
            self.sight_pub.publish(self._bridge.cv2_to_imgmsg(ImgFrame, 'bgr8'))
            self.point_pub.publish(self.target)
        except CvBridgeError, e:
            print e

    def rotate_callback(self,data):
        try:
            ImgFrame2 = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e

        ImgFrame2 = cv2.rotate(ImgFrame2, cv2.ROTATE_90_CLOCKWISE)#90度回転
        ImgFrame2 = cv2.flip(ImgFrame2, 1) #左右反転

        try:
            self.spectrum_pub.publish(self._bridge.cv2_to_imgmsg(ImgFrame2, 'bgr8'))
        except CvBridgeError, e:
            print e
        
if __name__ == "__main__":

    #inner = np.array([[614.470458984375, 0, 1036.625311183976, 0], [0, 730.07958984375, 773.5514250291162, 0], [0, 0, 1, 0]])#内部パラメーター 2048*1536
    #outer = np.dot(np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([[1,0,0,0],[0,1,0,88],[0,0,1,0],[0,0,0,1]]))外部パラメーター 2048*1536
    #inner = np.array([[307.6017761230469, 0, 500.9403417435533, 0], [0, 342.7832641601562, 379.5612670888731, 0], [0, 0, 1, 0]])#内部パラメーター 1024*768
    inner = np.array([[307.6017761230469, 0, 512, 0], [0, 342.7832641601562, 384, 0], [0, 0, 1, 0]])#内部パラメーター 1024*768
    outer = np.dot(np.array([[math.cos(math.radians(2)),0,-math.sin(math.radians(2)),0],[0,1,0,0],
        [math.sin(math.radians(2)),0,math.cos(math.radians(2)),0],[0,0,0,1]]),np.array([[1,0,0,0],[0,1,0,85],[0,0,1,0],[0,0,0,1]]))#外部パラメーター 1024*768
    #mtx = np.array([[898.740576, 0, 1047.656497], [0, 899.086181, 771.153764], [0, 0, 1]])#camera_matrix 2048*1536
    #dist = np.array([-0.202203, 0.029082, 0.000228, -0.0005, 0])#distortion coefficient 2048*1536
    mtx = np.array([[437.7949802332059, 0, 502.174798617283], [0, 437.0115481181825, 380.0865280376445], [0, 0, 1]])#camera_matrix 1024*768
    dist = np.array([-0.2171050089304354, 0.03583959522345996, 0.0009684942829751926, 0.0006630572393120683, 0])#distortion coefficient 1024*768

    fontcolor = (255,255,255)
    fontface  = cv2.FONT_HERSHEY_SIMPLEX
    fontthick = 2

    print('Enter current distance(m)')
    dis = float(input())
    i = dis * 1000 + 111.5

    world = np.array([[0],[(i + 93) * math.tan(math.radians(2.85))],[i],[1]])
    rect1 = np.array([[i * math.tan(math.radians(1.2))],[(i + 93) * math.tan(math.radians(2.85))],[i],[1]])
    rect2 = np.array([[-i * math.tan(math.radians(1.2))],[-(i + 93) * math.tan(math.radians(5.75))],[i],[1]])
    a = np.dot(inner,outer)

    b = project_n_undistort(a,world)
    b_3 = project_n_undistort(a,np.array([[0],[3204.5 * math.tan(math.radians(2.85))],[3111.5],[1]]))
    c = project_n_undistort(a,rect1)
    c2 = project_n_undistort(a,rect2)
    

    rospy.init_node('sight')
    sight = Sight()

    rospy.loginfo('check in viewer')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
#123
