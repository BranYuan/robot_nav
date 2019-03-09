#!/usr/bin/python2.7
# coding:utf-8
import SerialCmd
import time
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    enables_motor = [0x01,0x10,0x46,0x57,0x00,0x01,0x02,0x00,0x01,0x4d,0xb3]    #电机使能
    disables_motor = [0x01,0x10,0x46,0x57,0x00,0x01,0x02,0x00,0x00,0x8c,0x73]   #电机取消使能
    speeds_cw =       [0x01,0x10,0x44,0x20,0x00,0x02,0x04,0x27,0x10,0x00,0x00,0xf9,0xc5]    #电机正转速度
    speeds_ccw =      [0x01,0x10,0x44,0x20,0x00,0x02,0x04,0xd8,0xf0,0xff,0xff,0xc9,0x97]    #电机反转速度
    
    ser_4G = SerialCmd.SerialCmd(port='/dev/ttyS4')     #4G信号通信端口
    ser = SerialCmd.SerialCmd()     #下位机通信端口
    ser_left = SerialCmd.SerialCmd(port='/dev/ttyS2')   #左电机通信端口
    ser_right = SerialCmd.SerialCmd(port='/dev/ttyS3')  #右电机通信端口
    
    print ('listenning:')
    
    auto_flag = False   #自动运行标志，True为自动运行模式，False为遥控模式
    
    cur_time = time.strftime("%y-%m-%d",time.localtime(time.time()))
    #log_file = open('/home/guardrobot/guardRobot/log_file/'+cur_time+'.txt','a')
    log_file = open('/home/sweet/github_store/sweet_robot/4G_controller/log_file'+cur_time+'.txt','a')
    cmd_data = None     #接收控制命令变量

    #打开端口
    if not ser_4G.serial.isOpen():
        ser_4G.serial.open_port()
    #若端口打开，且有数据输入，则接收，并记录与logfile中
    if ser_4G.serial.isOpen():
        if ser_4G.serial.inWaiting():
            cmd_data = ser_4G.receive_info_str()
            if log_file:
                log_file.write(time.strftime("%y-%m-%d-%H:%M:%S",time.localtime(time.time()))+'----'+cmd_data)
        else:
            cmd_data = None
    else:
        cmd_data = None
    
    print(cmd_data)
    if cmd_data:
        #以*开头，#结尾，提取命令字段
        head = cmd_data.find("*")
        end = cmd_data.find("#")
        if end > head >-1:
            cmd_data = cmd_data[head:end+1]
        
        #下位机控制命令发送
        if not ser.serial.isOpen():
            ser.open_port()
        if ser.serial.isOpen():
            ser.send_cmd(cmd_data)


        #停止
        elif cmd_data == '*STOP#':
            auto_flag = False
            if not ser_left.serial.isOpen():
                ser_left.open_port()
            if ser_left.serial.isOpen():
                ser_left.send_cmd(disables_motor)
                log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(disables_motor)
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
                log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_cw)
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
                log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_ccw)
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
                log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_cw)
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
                log_file.write('----cmd_left_motor sended')
            if not ser_right.serial.isOpen():
                ser_right.open_port()
            if ser_right.serial.isOpen():
                ser_right.send_cmd(enables_motor)
                time.sleep(0.1)
                ser_right.send_cmd(speeds_ccw)
                log_file.write('----cmd_right_motor sended')
        
        #自动运行
        elif cmd_data == "AUTO_RUN":
            auto_flag = True
        print('end')
    if log_file:
        log_file.write("\n")
        log_file.close

    ser_4G = SerialCmd.SerialCmd(port='/dev/ttyS4')     #4G信号通信端口
    ser = SerialCmd.SerialCmd()     #下位机通信端口
    ser_left = SerialCmd.SerialCmd(port='/dev/ttyS2')   #左电机通信端口
    ser_right = SerialCmd.SerialCmd(port='/dev/ttyS3')  #右电机通信端口
    
    if ser_4G.serial.isOpen():
        ser_4G.serial.close()
    if ser.serial.isOpen():
        ser.serial.close()
    if ser_left.serial.isOpen():
        ser_left.serial.close()
    if ser_right.serial.isOpen():
        ser_right.serial.close()
