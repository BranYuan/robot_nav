#!/usr/bin/env python
# _*_ coding: utf-8 _*_

"""
Created on 2019-3-7
视觉识别发布器：发布识别结果

@author:Bran
"""

import rospy
from std_msgs.msg import Float32

def callback(data):
    result = data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(result))
    return result

def motor_controller():

    rospy.init_node('motor_controller',anonymous = True)
    rospy.Subscriber('img_status', Float32, callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
