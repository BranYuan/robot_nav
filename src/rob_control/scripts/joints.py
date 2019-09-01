#!/usr/bin/env python
#__*__coding:utf-8__*__

'''
主要功能：封装控制关节的代码
主要函数：
# __init__ 类的初始化
# set_current_position(self, position):发送关节控制命令
'''
import serial
import roslib
import rospy
from math import pi as PI, degrees, radians

class Joint:
    def __init__(self, name, nr_joint):
        self.name = name
        self.nr_joint = nr_joint

    def set_current_position(self):
        # rospy.wait_for_service('/arm/joint_write')
        try:
            pass
        except rospy.ServiceException, e:
            print 'Service call failed:%s'%e

    def set_current_position(self, position):
        # rospy.wait_for_service('/arm/joint_write')
        try:
            rospy.loginfo('====================')
        except rospy.ServiceException, e:
            print 'Service call failed %s'%e
if __name__ == '__main__':
    joint = Joint('arm',6)
