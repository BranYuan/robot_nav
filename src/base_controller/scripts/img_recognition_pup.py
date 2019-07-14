#!/usr/bin/env python
# _*_ coding: utf-8 _*_

"""
Created on 2019-3-7
视觉识别发布器：发布识别结果

@author:Bran
"""

import rospy
from std_msgs.msg import Float32
import cv2
import numpy as np
import pandas as pd
import time



# 图像处理，参数s为图像识别阈值，取值0-255，默认s=15，返回新图像对象和图像灰度位置平均百分比
def process_frame(frame, s = 5):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    sensitivity = s
    lower_white = np.array([0,0,255-sensitivity], dtype=np.uint8)
    upper_white = np.array([255,sensitivity,255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_white, upper_white)
    #result = cv2.bitwise_and(frame, frame, mask= mask)
    new_frame = cv2.bitwise_and(frame, frame, mask= mask)    


    gray = cv2.cvtColor(new_frame, cv2.COLOR_RGB2GRAY)
    binary_output = np.zeros_like(gray)
    binary_output[gray != 0] = 1

    height, width = gray.shape
    xy = pd.DataFrame(np.dstack((gray.nonzero()[0],gray.nonzero()[1]))[0],columns=['y','x'])

    result = xy[xy['y'] > height/2]['x'].mean()/width
    #upper_percentage = xy[xy['y'] > height/2]['x'].mean()/width
    #lower_percentage = xy[xy['y'] < height/2]['x'].mean()/width
    #print(xy[xy['y'] > height/2]['x'].mean()/width)
    #print(xy[xy['y'] < height/2]['x'].mean()/width)

    #result = cv2.putText(result, xy[xy['y'] > height/2]['x'].mean()/width, (2,2), 0, 1, (0,0,0), 2, cv2.LINE_AA)
    #result = cv2.putText(result, xy[xy['y'] < height/2]['x'].mean()/width, (2,4), 0, 1, (0,0,0), 2, cv2.LINE_AA)

    return new_frame, result



def status_pup():
    # 发布者：img_recognition
    pup = rospy.Publisher('img_status',Float32,queue_size = 1)
    # 发布节点：img_status
    rospy.init_node('img_recognition_pup',anonymous = True)
    #发布频率：10Hz
    rate = rospy.Rate(4)
    
    #生成图像显示窗口
    cv2.namedWindow("preview")
    #参数调节摄像头，如果只有一个摄像头 则为0
    vc = cv2.VideoCapture(0)
    #摄像头开启
    if vc.isOpened():
        rval, frame = vc.read()
    else:
        rval = False
    
    
    while not rospy.is_shutdown():
        if rval:
            #读取图像
            rval, frame = vc.read()
            #调用函数，处理图像
            frame,result = process_frame(frame,30)
            # 显示摄像头图形
            cv2.imshow('preview',frame)
            key = cv2.waitKey(20)
            if key == 27:
                break
        #如果没有读到图像，result小于0
        else:
            result = -1
        loginfo = "%s result:" % rospy.get_time() + str(result)
        # 记录log信息
        rospy.loginfo(loginfo)
        #发布结果
        pup.publish(result)
        rate.sleep()
    #关闭图像显示窗口
    cv2.destroyWindow("preview")
    #释放摄像头
    vc.release()

if __name__ == '__main__':
    try:
        status_pup()
    except rospy.ROSInterruptException:
        pass
