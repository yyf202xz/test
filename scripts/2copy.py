#!/usr/bin/env python
## coding: UTF-8

import rospy
import cv2
import numpy as np
import fps
import arucodetect
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

i = 0
data = []
data2 = []

ratio =[0]*2

fontcolor = (255,255,255)
window_name = 'Capture'
fontthick = 2

class Detect(object):
    def __init__(self):
        self._image_sub = rospy.Subscriber('/image_raw', Image, self.callback)
        self._bridge = CvBridge()
            
    def callback(self, dota):
        global i,data,data2,ratio,gFrameRate,fontcolor,window_name,fontface,fontthick
        try:
            ImgFrame = self._bridge.imgmsg_to_cv2(dota, 'bgr8')
            print('Enter current distance(m). Exit by entering any others')
            #try:
            dis = input()
            while 1:                       
                dimensions = ImgFrame.shape[:2]
                dimensions = dimensions[::-1]
                pos2, ImgFrame = arucodetect.arucodetect(ImgFrame)
                ratio[0] = float(pos2[0])/dimensions[0]
                ratio[1] = float(pos2[1])/dimensions[1]
                #解像度表示
                
                cv2.putText(ImgFrame, str(dimensions), (100,25), fontface, 1.0, fontcolor, fontthick , cv2.LINE_AA)
                #フレームレート取得
                fps = gFrameRate.get()           
                fps_str = '%4d' % fps
                cv2.putText(ImgFrame, fps_str, (10,25), fontface, 1.0, fontcolor, fontthick , cv2.LINE_AA)
                print 5
                cv2.imshow('Capture',ImgFrame)
                print 1
                print 'x='+str(ratio[0]),'y='+str(ratio[1])

                #key = cv2.waitKey(10)

                if  key == ord('s'):
                    dis_n_pos_n_ratio = np.concatenate((dis, pos2, ratio), axis=None)
                    dis_n_pos_n_ratio2 = np.round(dis_n_pos_n_ratio,2)             

                    print(dis_n_pos_n_ratio2)   

                    data = np.insert(data, i, dis_n_pos_n_ratio, axis=0)
                    data2 = np.insert(data2, i, dis_n_pos_n_ratio2, axis=0)

                    i = i+1

                    data = data.reshape(i, 5)
                    data2 = data2.reshape(i, 5)

                    x_dis = data[:,0]#X軸(横軸)データ作成
                    y_x_pos = data[:,1]#Y軸(縦軸)データ作成
                    y_y_pos = data[:,2]#Y軸(縦軸)データ作成

                    plt.subplot(2, 1, 1)#グラフを表示
                    plt.plot(x_dis, y_x_pos, '.-')
                    plt.ylabel('Position_x')

                    plt.subplot(2, 1, 2)
                    plt.plot(x_dis, y_y_pos, '.-')
                    plt.xlabel('Distance(m)')
                    plt.ylabel('Position_y')
                    plt.ion()
                    plt.show()
                    break
                
                elif key == ord('q'):
                    break
            '''
            except NameError:
                print 'over'
                data = np.insert(data, 0, (0,0,0,dimensions[0],dimensions[1]), axis=0)
                np.savetxt("calidata.csv", data, delimiter=",")
                print(' |Distance|x(pixel)|y(pixel)|x(ratio)|y(ratio)| ')
                print data2
            '''
            cv2.destroyAllWindows()
        except CvBridgeError, e:
            print e
        
    
if __name__ == '__main__':
   
    rospy.init_node('Detect')
    data = np.array(data)
    data2 = np.array(data2)
    gFrameRate = fps.FrameRate()
    fontface  = cv2.FONT_HERSHEY_SIMPLEX
    detect = Detect()
    #cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


    