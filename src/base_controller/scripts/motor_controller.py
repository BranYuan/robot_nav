#!/usr/bin/env python
# _*_ coding: utf-8 _*_

"""
Created on 2019-3-7
马达等硬件控制器：订阅speed数据并控制电机

@author:Bran
"""

import rospy
import SerialCmd
import time
from std_msgs.msg import Float32, Float32MultiArray

MAX_SPEED = 2500    # 电机最大速度
enables_motor = [0x01,0x10,0x46,0x57,0x00,0x01,0x02,0x00,0x01,0x4d,0xb3]    #电机使能
disables_motor = [0x01,0x10,0x46,0x57,0x00,0x01,0x02,0x00,0x00,0x8c,0x73]   #电机取消使能
speed_r_cmd = [0x01,0x10,0x44,0x20,0x00,0x02,0x04,0x13,0x88,0x00,0x00,0x76,0x1a]    #右电机速度,默认500
speed_l_cmd = [0x01,0x10,0x44,0x20,0x00,0x02,0x04,0xec,0x78,0xff,0xff,0x47,0x8d]    #左电机速度，默认-500
speeds_cw =   [0x01,0x10,0x44,0x20,0x00,0x02,0x04,0x61,0xa8,0x00,0x00,0x6c,0xa8]    #电机正转速度,默认500
speeds_ccw =  [0x01,0x10,0x44,0x20,0x00,0x02,0x04,0x9e,0x58,0xff,0xff,0x5d,0x3f]    #电机反转速度，默认-500
auto_flag = False   #自动运行标志，True为自动运行模式，False为遥控模式


ser_4G = SerialCmd.SerialCmd(port='/dev/ttyS4')     #4G信号通信端口
ser = SerialCmd.SerialCmd()     #下位机通信端口
ser_left = SerialCmd.SerialCmd(port='/dev/ttyS2')   #左电机通信端口
ser_right = SerialCmd.SerialCmd(port='/dev/ttyS3')  #右电机通信端口


# 遥控运行时，左右轮速度的计算函数
def manul_speed_calculate(max_speed,speeds_cw,speeds_ccw):
    if type(max_speed) is int and max_speed >= 0 and type(speeds_cw) is list and len(speeds_cw) == 13 and type(speeds_ccw) is list and len(speeds_ccw) == 13:
        if max_speed > MAX_SPEED:
            max_speed = MAX_SPEED
        max_speed *= 10
        speeds_cw[7] = max_speed >> 8 & 0xff
        speeds_cw[8] = max_speed & 0xff
        max_speed = 0xffff - max_speed + 1
        speeds_ccw[7] = max_speed >> 8 & 0xff
        speeds_ccw[8] = max_speed & 0xff
        crc_calculate(speeds_cw)
        crc_calculate(speeds_ccw)


#speed_l 和 speed_r的命令的list合成,用于自动速度控制模式时的速度的计算
def speed_calculate(speed_l,speed_r):
    speed_l = int(speed_l*10)     #左轮电机速度的10倍
    speed_r = int(speed_r*10)     #右轮电机速度的10倍
    #print(str(speed_l) + '-------' + str(speed_r))
    speed_l = 0xffff - speed_l +1 #右轮与左轮转向相反
    speed_l_cmd[8] = speed_l & 0xff         #速度低八位
    speed_l_cmd[7] = (speed_l >> 8) & 0xff    #速度高八位
    speed_r_cmd[8] = speed_r & 0xff
    speed_r_cmd[7] = (speed_r >> 8) & 0xff
    crc_calculate(speed_l_cmd)        #校验位计算
    crc_calculate(speed_r_cmd)

#校验位计算函数
def crc_calculate(data_list):
    crc = 0xffff
    if type(data_list) is list and len(data_list)>2:
        for i  in data_list[:-2]:
            crc ^= i
            for j in range(8):
                if (crc & 1):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        data_list[-2] =  crc & 0xff
        data_list[-1] =  crc >>8 & 0xff


# 自动运行
def auto_run(speed_l, speed_r):
    cur_time = time.strftime("%y-%m-%d",time.localtime(time.time()))
    #log_file = open('/home/guardrobot/guardRobot/log_file/'+cur_time+'.txt','a')
    log_file = open('/home/sweet/github_store/sweet_robot/4G_controller/log_file'+cur_time+'.txt','a')
    cmd_data = None     #接收控制命令变量
    global auto_flag
    #打开端口
    if not ser_4G.serial.isOpen():
        ser_4G.serial.open_port()
    #若端口打开，且有数据输入，则接收，并记录与logfile中
    if ser_4G.serial.isOpen():
        if ser_4G.serial.inWaiting():
            cmd_data = ser_4G.receive_info_str()
            ser_4G.send_cmd(cmd_data)
            if log_file:
                log_file.write(time.strftime("%y-%m-%d-%H:%M:%S",time.localtime(time.time()))+'----'+cmd_data)
        else:
            cmd_data = None
    else:
        cmd_data = None
    #print(cmd_data)
    if cmd_data:
        print(cmd_data)
        #以*开头，#结尾，提取命令字段
        head = cmd_data.find("*")
        end = cmd_data.find("#")
        if end > head >-1:
            cmd_data = cmd_data[head:end+1]
        
        # 通过遥控改变最大运行速度
        #解析命令：SPEEDXXXX,XXXX为最大速度值
        if cmd_data.find("SPEED") == 1 and len(cmd_data) == 11:
            cmd_data = cmd_data[6:10]
            int_flag = True
            for i in cmd_data:
                if not "0" <= i <= "9":
                    int_flag =False
            if int_flag:
                print("speed\n")
                max_speed = int(cmd_data)
                # 计算速度指令
                manul_speed_calculate(max_speed, speeds_cw, speeds_ccw)
                # 开4G串口
                if not ser_4G.serial.isOpen():
                    ser_4G.serial.open_port()
                if ser_4G.serial.isOpen:
                    ser_4G.send_cmd("*SPEED_OK#")
                    if log_file:
                        log_file.write('\nset speed to:'+ cmd_data +' ----speeds_cw:')
                        for i in speeds_cw:
                            log_file.write(str(hex(i)) + "  ")
                        log_file.write("----speeds_ccw:")
                        for i in speeds_ccw:
                            log_file.write(str(hex(i)) + "  ")


        #下位机控制命令发送
        if not ser.serial.isOpen():
            ser.open_port()
        if ser.serial.isOpen():
            ser.send_cmd(cmd_data)
        #停止
        if cmd_data == '*STOP#':
            auto_flag = False
            if not ser_left.serial.isOpen():
                ser_left.open_port()
            if ser_left.serial.isOpen():
                ser_left.send_cmd(disables_motor)
                if log_file:
                    log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(disables_motor)
                if log_file:
                    log_file.write('----cmd_right_motor sended')
        
        #前进
        elif cmd_data == '*AHEAD#':
            auto_flag = False
            if not ser_left.serial.isOpen():
                ser_left.open_port()
            if ser_left.serial.isOpen():
                ser_left.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_left.send_cmd(speeds_ccw)
                if log_file:
                    log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_cw)
                if log_file:
                    log_file.write('----cmd_right_motor sended')
        
        #后退
        elif cmd_data == '*BACK#':
            auto_flag = False
            if not ser_left.serial.isOpen():
                ser_left.open_port()
            if ser_left.serial.isOpen():
                ser_left.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_left.send_cmd(speeds_cw)
                if log_file:
                    log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_ccw)
                if log_file:
                    log_file.write('----cmd_right_motor sended')
        
        #左转
        elif cmd_data == '*LEFT#':
            auto_flag = False
            if not ser_left.serial.isOpen():
                ser_left.open_port()
            if ser_left.serial.isOpen():
                ser_left.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_left.send_cmd(speeds_cw)
                if log_file:
                    log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_cw)
                if log_file:
                    log_file.write('----cmd_right_motor sended')
        
        #右转
        elif cmd_data == '*RIGHT#':
            auto_flag = False
            if not ser_left.serial.isOpen():
                ser_left.open_port()
            if ser_left.serial.isOpen():
                ser_left.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_left.send_cmd(speeds_ccw)
                if log_file:
                    log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_ccw)
                if log_file:
                    log_file.write('----cmd_right_motor sended')
        
        #自动运行
        elif cmd_data == "*AUTO_RUN#":
            auto_flag = True
            if log_file:
                log_file.write('----auto_run')
        # print('end')
    
    
    #自动运行
    speed_calculate(speed_l,speed_r)
    if auto_flag:
        # print('--------------------------------')
        if not ser_left.serial.isOpen():
            ser_left.open_port()
        if ser_left.serial.isOpen():
            ser_left.send_cmd(enables_motor)
            time.sleep(0.1)
            ser_left.send_cmd(speed_l_cmd)
        if not ser_right.serial.isOpen():
            ser_right.open_port()
        if ser_right.serial.isOpen():
            ser_right.send_cmd(enables_motor)
            time.sleep(0.1)
            ser_right.send_cmd(speed_r_cmd)
        print("auto_run:" + str(speed_l) + str(speed_r))
    
    if log_file:
        if cmd_data:
            log_file.write("\n")
        log_file.close
    
def callback(data):
    cur = time.time()
    speed_l = data.data[0]
    speed_r = data.data[1]
    auto_run(speed_l, speed_r) 
    # rospy.loginfo("I heard %s", str(speed_l) + "----" + str(speed_r))
    # print(str(time.time()-cur)) 

def motor_controller():
    print("start")
    #创建motor_controller节点
    rospy.init_node('motor_controller',anonymous = True)
    #订阅话题:speed
    rospy.Subscriber('speed', Float32MultiArray, callback)
    rospy.spin()
    if ser_4G.serial.isOpen():
        ser_4G.serial.close()
    if ser.serial.isOpen():
        ser.serial.close()
    if ser_left.serial.isOpen():
        ser_left.serial.close()
    if ser_right.serial.isOpen():
        ser_right.serial.close()



if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
