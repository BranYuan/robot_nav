#!/usr/bin/env python
# __*__coding:utf-8__*__

import cv2
import numpy as np

url = "rtsp://admin:@192.168.1.52:554/h264/ch1/main/av_stream"

cap = cv2.VideoCapture(url)

# cascade = cv2.CascadeClassifier("./haarcascade_frontalface_alt.xml")
while True:
    (ret,frame)=cap.read()
    frame=cv2.flip(frame,0)
    gray_frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    '''
    rects=cascade.detectMultiScale(gray_frame,scaleFactor = 1.2, minNeighbors = 3, minSize = (32, 32))
    if len(rects)>0: #如果>0说明检测到人了
        for rect in rects:
            x,y,w,h=rect
            p1,p2=(x,y),(x+w,y+h)
            cv2.rectangle(frame,p1,p2,color=(0,0,255),thickness=2)
    '''
    cv2.imshow('Video', gray_frame)
    cv2.waitKey(1)

