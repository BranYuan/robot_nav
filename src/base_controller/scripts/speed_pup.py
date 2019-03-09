#!/usr/bin/env python
# _*_ coding: utf-8 _*_

"""
Created on 2019-3-7
速度发布器：订阅视觉识别结果，并发布左右电机速度

@author:Bran
"""

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from math import isnan

MAX_SPEED = 2500 # 电机最大限制速度
MIN_SPEED = 400 #电机最小速度限制
DELTA = 200     # PID算法运行时，速度变化的基数
GOAL = 0.5      # 目标位置

# PI调节器默认参数定义
PID_ARG = {'kp':4.0, 'ki':0.1, 'kd':0.1, 'ek':0.0, 'ek1':0.0, 'ek2':0, 'uk': 0.0, 'uk1':0.0, 'adjust': 0}


# pid系数生成函数
def motor_pid(result = 0.5, PID_ARG = PID_ARG):
   
    if isnan(result):
        result = 0.5
    PID_ARG['ek'] = GOAL - result

    if abs(PID_ARG['ek']) < 0.05:
        adjust = 0
    else:
        PID_ARG['uk'] = PID_ARG['kp'] * PID_ARG['ek'] + PID_ARG['ki'] * PID_ARG['ek2'] + PID_ARG['kd'] * (PID_ARG['ek'] - PID_ARG['ek1'])
        adjust = PID_ARG['uk']
        PID_ARG['uk1'] = PID_ARG['uk']
        PID_ARG['ek1'] = PID_ARG['ek']
        PID_ARG['ek2'] += PID_ARG['ek']
    
    PID_ARG['adjust'] = adjust


# 左右轮电机速度计算
def speed_calc(result):
    motor_pid(result)
    speed_delta = DELTA * PID_ARG['adjust'] #速度调整量
    speed_l = speed_r = MAX_SPEED #左右电机转速
    if speed_delta < 0:
        speed_l += speed_delta
    else:
        speed_r -= speed_delta

    if speed_l < MIN_SPEED:
        speed_l = MIN_SPEED
    if speed_r < MIN_SPEED:
        speed_r = MIN_SPEED
    return speed_l, speed_r
        



# 订阅器回调函数
def callback(data, pup):
    result = data.data
    speed = Float32MultiArray()
    # 计算左右轮速度
    speed.data = speed_calc(result)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(result))
    # print(speed)
    # print(speed.data)
    pup.publish(speed)
    return result


# 主函数，节点，订阅器，发布器生成
def motor_controller():
    rospy.init_node('speed_pup',anonymous = True)
    pup = rospy.Publisher('speed', Float32MultiArray, queue_size = 2)
    rospy.Subscriber('img_status', Float32, callback, pup)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass