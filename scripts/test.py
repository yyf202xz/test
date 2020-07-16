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
# ここでは青色を抽出するので120±20を閾値とした
LOW_COLOR = np.array([100, 75, 75])
HIGH_COLOR = np.array([140, 255, 255])

# 抽出する青色の塊のしきい値
AREA_RATIO_THRESHOLD = 0.005

class ColorExtract(object):

    def __init__(self):
            self._blue_pub = rospy.Publisher('blue_image', Image, queue_size=1)
            self._circle_pub = rospy.Publisher('circle', Image, queue_size=1)
            self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            self._image_sub = rospy.Subscriber('/image_raw', Image, self.callback)
            self._bridge = CvBridge()
            self._vel = Twist()

    def get_colored_area(self, cv_image, lower, upper):
        # 高さ，幅，チャンネル数
        h,w,c = cv_image.shape

        # hsv色空間に変換
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 色を抽出する
        mask_image = cv2.inRange(hsv_image, lower, upper)

        # 画像をくり抜く
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)

         # 輪郭抽出
        contours,hierarchy = cv2.findContours(mask_image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
        
        # 面積を計算
        areas = np.array(list(map(cv2.contourArea,contours)))
        area = cv2.countNonZero(mask_image)

        if len(areas) == 0 or np.max(areas) / (h*w) < AREA_RATIO_THRESHOLD:
            # 見つからなかったら
            print("the area is too small") 
            err_x = 0
            err_y = 0
            area = 0
            circle = cv_image

        else:
            # 面積が最大の塊の重心を計算し返す
            max_idx = np.argmax(areas)
            max_area = areas[max_idx]
            result = cv2.moments(contours[max_idx])
            x = int(result["m10"]/result["m00"])
            y = int(result["m01"]/result["m00"])
            pos=x,y

            #輪郭描画
            cnt = contours[max_idx]
            image_with_contours=cv2.drawContours(cv_image, [cnt], 0, (0, 255, 0), 3)

            # ズレ値を表示
            err_x = x-(w/2)
            err_y = y-(h/2)

            # 抽出した座標に丸を描く
            circle = cv2.circle(image_with_contours,pos,10,(0,0,255),-1)

        return (h, w, c, err_x, err_y, area, extracted_image,circle)
     
    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        h, w, c, err_x, err_y, blue_area, blue_image, circle = self.get_colored_area(cv_image, LOW_COLOR, HIGH_COLOR)
        try:
            self._blue_pub.publish(self._bridge.cv2_to_imgmsg(blue_image, 'bgr8'))
            self._circle_pub.publish(self._bridge.cv2_to_imgmsg(circle, 'bgr8'))
            self._vel.linear.x = err_x
            self._vel.linear.y = err_y
            self._vel_pub.publish(self._vel)
        except CvBridgeError, e:
            print e
        rospy.loginfo('blue=%d,err_x=%d,err_y=%d,h=%d,w=%d,c=%d' % (blue_area, err_x, err_y, h, w, c))
    
if __name__ == '__main__':
    rospy.init_node('color_extract')
    color = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
