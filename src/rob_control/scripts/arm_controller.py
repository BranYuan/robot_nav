#!/usr/bin/env python
# __*__coding:utf-8__*__

import sys
import copy
import rospy
import actionlib
import threading
import SerialCmd
import socket
import os
import re
import time
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from diagnostic_msgs.msg import *
from math import pi as PI, degrees, radians
from joints import Joint
import minimalmodbus as modbus


G_JOINT_MSG = JointState()
SER_ARM = modbus.Instrument(port='/dev/ttyS6', slaveaddress=2)

G_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
G_POSITION = [radians(i) for i in [-11.686,64.865,-15.943,55.433,47.906,-21.150]]

G_SER_IN_USE = 0 # 判断串口是否被进程占用
HOME_FLAG = False # 机器人在原点标志
AUTO_FLAG = False # 自动采摘有效标志

DEGREE_OFFSET = 400 # 机器人寄存器不能写入负数，故角度偏移400
DEGREE_RATIO = 10        # 角度放大比例
DEBUG = False

GRIPPER_ON = 1 # 夹抓打开
GRIPPER_OFF = 0 # 夹抓关闭

GOAL_SIZE = 0.08 # 梨的尺寸
GOAL_POSITION = [0.0, -0.99398, 0.18418] # 梨目标所在位置
GOAL_FLAG = -1 # 是否有目标传入标志,1:目标传入，2:机械手抓取成功，3:抓取失败,0:待机械手准备好

#寄存器地址
ARM_READY_REG = 100
POINT1_OK_REG = 101
POINT2_OK_REG = 102
POINT_OK_REG = 106
POINT_CUR_REG = 107
POINT1_START_REG = 113
POINT2_START_REG = 119
COIL_Y15 = 0x0F



def arm_init():
    global G_SER_IN_USE
    global G_JOINT_MSG
    global SER_ARM
    global GOAL_POSITION
    global GOAL_FLAG
    
    rospy.init_node('arm_controller')
    G_JOINT_MSG.header = Header()
    G_JOINT_MSG.header.stamp = rospy.Time.now()
    G_JOINT_MSG.name = G_JOINTS
    G_JOINT_MSG.position = G_POSITION
    G_JOINT_MSG.velocity = []
    G_JOINT_MSG.effort = []
    SER_ARM.serial.baudrate = 115200
    SER_ARM.serial.timeout = 0.5
    SER_ARM.serial.write_timeout = 0.5
    
    rate = rospy.Rate(50.0)
    pup = rospy.Publisher('joint_states', JointState, queue_size = 5)
    # sub = rospy.Subscriber('joint_states', JointState, callback)
    ''' 
    while not read_reg(ARM_READY_REG):
        ## 等待机器人启动
        print 'waiting for arm'
        rospy.sleep(0.5)
    current_joints = read_joints(POINT1_START_REG, 6)
    while current_joints == None:
        current_joints = read_joints(POINT1_START_REG, 6)
        print 'waiting for read current position'
        rospy.sleep(0.5)
    G_JOINT_MSG.position = current_joints
    '''
    # 设置串口处理线程
    serial_thread = threading.Thread(target=serial_loop, name='SerialThread')
    # socket处理线程
    socket_thread = threading.Thread(target=socket_loop, name='SocketThread')
    #定义并启动发布者多线程
    pup_thread = threading.Thread(target=pup_thread_loop,args = (pup,), name='PubLoopThread')
    # 控制器线程
    act_server_thread = threading.Thread(target=act_server_thread_loop, name='ActLoopThread') 
    # 设置目标位置线程
    set_goal_thread = threading.Thread(target=set_goal_loop, name='SetGoalThread') 

    serial_thread.start()
    socket_thread.start()
    pup_thread.start()
    act_server_thread.start()
    set_goal_thread.start()
    print 'start thread'
    serial_thread.join()
    socket_thread.join()
    pup_thread.join()
    act_server_thread.join()
    set_goal_thread.join()
    # rospy.spin()

def pup_thread_loop(pup):
    # 发布者线程主循环
    # pup = rospy.Publisher('joint_states', JointState, queue_size = 5)
    while not rospy.is_shutdown():
        if not G_SER_IN_USE:
            G_JOINT_MSG.header.stamp = rospy.Time.now()
        pup.publish(G_JOINT_MSG)
        rospy.sleep(0.1)

def act_server_thread_loop():
    # arm_controller server线程主循环
    FollowController()
    rospy.spin()


def socket_loop():
    server_main()

def serial_loop():
    ## 串口命令处理进程
    ser_4G = SerialCmd.SerialCmd(port = '/dev/ttyS4')
    ser = SerialCmd.SerialCmd()
    global DEBUG
    init_position = copy.deepcopy(G_JOINT_MSG.position)  # 当记录机器人初始位姿，当收到debug模式开启时，用于初始化位姿
    if not ser_4G.serial.isOpen():
        ser_4G.open_port()
    if not ser.serial.isOpen():
        ser.open_port()
    while not DEBUG:
        ## 初始化主板
        try:
            if not ser.serial.isOpen():
                ser.open_port()
            if ser.serial.isOpen():
                ser.send_cmd('*BSDB#')
            print('Initting slave')
            if ser.serial.inWaiting():
                break
            else:
                rospy.sleep(2)
        except:
            pass
    print('start serial loop')
    while not rospy.is_shutdown():
        try:
            cmd_data = None
            if ser_4G.serial.isOpen():
                while not ser_4G.serial.inWaiting():
                    rospy.sleep(0.1)
                cmd_data = ser_4G.receive_info_str()
                ser_4G.send_cmd(cmd_data)
                if cmd_data:
                    try:
                        print(cmd_data)
                        head = cmd_data.find('*')
                        end = cmd_data.find('#')
                        if end > head > -1:
                            cmd_data = cmd_data[head:end+1]
                        if not ser.serial.isOpen():
                            ser.open_port()
                        if ser.serial.isOpen():
                            ser.send_cmd(cmd_data)
                        if cmd_data == '*AUTO_RUN#':
                            AUTO_FLAG = True
                        else:
                            AUTO_FLAG = False
                        if cmd_data == '*DEBUG_ON#':
                            DEBUG = True
                            G_JOINT_MSG.position = init_position
                        elif cmd_data == '*DEBUG_OFF#':
                            DEBUG = False
                    except:
                        print('cmd err')
        except:
            pass


def set_goal_loop():
    global GOAL_FLAG 
    # wait for RVIZ=======================
    rospy.loginfo('wait for RVIZ=======================')
    rospy.sleep(10)
    GOAL_FLAG = 0 # 机械手准备好接收目标点

    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化move group控制机械臂中的 arm
    arm = moveit_commander.MoveGroupCommander('arm')
    # 获取终端link名称
    end_effector_link = arm.get_end_effector_link()
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    arm.set_num_planning_attempts(5)
    # 设置位姿的允许误差
    arm.set_goal_position_tolerance(0.003)
    arm.set_goal_orientation_tolerance(0.005) 
    # 求home_work_joints 到 home_joints最短路径
    # traj = cal_multi_point_traj(arm, way_work_home)

    # 各固定点初始化
    home_joints = [radians(i) for i in [89.156,57.127,38.238,-6.354,96.253,0]]
    release_joints = [radians(i) for i in [89.154,50.331,13.980,-6.95,65.4,0]]
    medium_joints = [radians(i) for i in [-19.654,88.135,-5.039,4.160,78.831,0]]
    home_work_joints = [radians(i) for i in [-90,75.795,-24.434,5.566,-33.123,0]]
    # 关节值对应的POSE
    home_pose = cal_pose(0.0333, 0.848, 0.34435, 0.99835, -0.01376, 0.05507, 0.00819)
    release_pose = cal_pose(0.0333, 0.848, -0.1, 0.99829, 0.01759, 0.05479, 0.00994)
    medium_pose = cal_pose(0.50435, -0.22916, 0.34435, -0.56995, 0.82005, -0.00978, 0.05071)
    home_work_pose = cal_pose(0.0, -0.99398, 0.18418, 0.03735, 0.76177, -0.64627, 0.02561)
    # 初始化目标位置 
    goal = copy.deepcopy(home_work_pose)
    # 目标物存储位置
    release_list = []
    release = copy.deepcopy(release_pose)
    release.position.x -= (3 * GOAL_SIZE)
    x = release.position.x
    release_list.append(copy.deepcopy(release))
    for i in range(1, 5):
        for i in range(1, 5):
            release.position.x += 1.5 * GOAL_SIZE
            release_list.append(copy.deepcopy(release))
        release.position.y -= 1.5 * GOAL_SIZE
        release.position.x = x
    
    # home点到work点路径点
    way_point_home_to_work = [home_pose, medium_pose]# , home_work_pose]
    # 笛卡尔路径规划，home to work
    G_JOINT_MSG.position = home_joints
    rospy.sleep(2)
    arm.set_start_state_to_current_state()
    traj_home_to_work = cal_cartesian_path(arm, way_point_home_to_work)
    traj_home_to_work = traj_home_to_work.joint_trajectory
    # 缩减路径点密集度
    decrease_traj_len(traj_home_to_work, 10)
    # 笛卡尔路径规划 work to home
    traj_work_to_home = (copy.deepcopy(traj_home_to_work))
    traj_work_to_home.points.reverse()
    
    ## 初始化存储位轨迹列表
    traj_release_goals = []
    traj_release_backs = []
    G_JOINT_MSG.position = copy.deepcopy(traj_work_to_home.points[-1].positions)
    rospy.sleep(2)
    
    for i in release_list:
        traj_go, traj_back = get_goal_traj(arm, i, 'z', 1)
        traj_go = traj_go.joint_trajectory
        traj_back = traj_back.joint_trajectory
        traj_release_goals.append(copy.deepcopy(traj_go))
        traj_release_backs.append(copy.deepcopy(traj_back))

    ## 设置目标位置进程主循环
    while not read_reg(ARM_READY_REG):
        ## 等待机器人启动
        print 'waiting for arm:' + str(read_reg(ARM_READY_REG))
        rospy.sleep(2)
    if  not DEBUG:
        current_joints = read_joints(POINT_CUR_REG, 6)
        while current_joints == None:
            current_joints = read_joints(POINT1_START_REG, 6)
            print 'waiting for read current position'
            rospy.sleep(2)
        G_JOINT_MSG.position = current_joints
        # print ([degrees(i) for i in current_joints])

    
    '''
    while not traj:
        traj = cal_multi_point_traj(arm, way_work_home)
        print 'Repanning way from work to home'
        rospy.sleep(2)
    '''
    # 回工作原点
    print('go to work home')
    traj_to_work_home = copy.deepcopy(traj_home_to_work)
    temp_point = traj_to_work_home.points[-1]
    traj_to_work_home.points = []
    traj_to_work_home.points.append(temp_point)
    execute_trajectory(traj_to_work_home)
    # 执行运动路径
    print('Now at home')
    while not rospy.is_shutdown():
        i = 0
        while i < len(traj_release_goals):
            while not GOAL_FLAG == 1:
                time.sleep(2)
            goal.position.x = GOAL_POSITION[0]
            goal.position.y = GOAL_POSITION[1]
            goal.position.z = GOAL_POSITION[2]
            try:
                traj_go, traj_back = get_goal_traj(arm, goal, 'y', 1)
                traj_go = traj_go.joint_trajectory
                traj_back = traj_back.joint_trajectory
                print 'go to goal '
                # 到目标点
                write_coil(COIL_Y15, GRIPPER_ON)
                while not execute_trajectory(traj_go):
                    pass
                rospy.sleep(9)
                write_coil(COIL_Y15, GRIPPER_OFF)                
                rospy.sleep(1.5)
                # 回工作原点
                print('go back to work home')
                while not execute_trajectory(traj_back):
                    pass
                # 抓取目标成功标志
                GOAL_FLAG = 2
                # 到home点 
                print('go home')
                while not execute_trajectory(traj_work_to_home):
                    pass
                # 存储位
                print('go to  release')
                while not execute_trajectory(traj_release_goals[i]):
                    pass
                rospy.sleep(4)
                write_coil(COIL_Y15, GRIPPER_ON)
                # 回home点
                print('go home')
                while not execute_trajectory(traj_release_backs[i]):
                    pass
                # 到工作原点
                print('go  to work')
                while not execute_trajectory(traj_home_to_work):
                    pass
                print 'Done'
                i += 1
            except:
                GOAL_FLAG = 3
                print 'can not go to goal position'
            # rospy.sleep(5)

def cal_pose(x,y,z,ax,ay,az,aw, reference_frame = 'base_link'):
    # 根据参数返回 geometry_msgs.msg.Pose()
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = ax
    target_pose.orientation.y = ay
    target_pose.orientation.z = az
    target_pose.orientation.w = aw
    return target_pose

def get_goal_traj(arm, goal, axis = 'y', direction = -1):
    ## 计算但前位姿到目标位姿的轨迹
    ## 主要取当前位姿，安全位姿，目标位姿三个值为轨迹
    ## 用于规划安全点到达取放点时的轨迹规划
    try:
        goal_temp = copy.deepcopy(goal)
        arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(goal_temp)
        traj = arm.plan()
        del traj.joint_trajectory.points[0:-1]
        # 设置机械臂终端运动目标位姿的安全位姿
        if axis == 'x':
            goal_temp.position.x += (0.1*direction)
        elif axis == 'y':
            goal_temp.position.y += (0.1*direction)
        elif axis == 'z':
            goal_temp.position.z += (0.1*direction)

        arm.set_pose_target(goal_temp)
        traj_safety = arm.plan()
        traj.joint_trajectory.points[0] = traj_safety.joint_trajectory.points[-1]
        # 取原路返回的轨迹
        traj_back = copy.deepcopy(traj)
        traj_back.joint_trajectory.points.reverse()
        return traj, traj_back
    except:
       return None, None


def cal_min_traj(group, end_joints, start_pose=None, end_pose=None):
    group.set_start_state(robot_state)
    group.set_joint_value_target(end_joints)
    try:
        traj = arm.plan()
    except:
        return False
    for i in range(10):
        try:
            traj0 = arm.plan()
            if len(traj0.joint_trajectory.points)<len(traj.joint_trajectory.points):
                traj = traj0
        except:
            pass
    return traj

## 多路径点规划
def cal_multi_point_traj(group, way_points):
    len_way = len(way_points)
    traj_list = []
    if len_way > 1:
        for i in range(len_way-1):
            traj = cal_min_traj(group, way_points[i],way_points[i+1])
            if traj0:
                traj_list += traj.joint_trajectory.points
            else:
                return False
        traj.joint_trajectory.points = traj_list
        return traj
    else:
        return False
def cal_cartesian_path(group, way_points, step_len = 0.005, maxtries=100):
    # 笛卡尔路径规划
    attempts = 0 # 已经尝试规划次数
    fraction = 0.0 # 路径覆盖率
    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路径点
    while fraction < 1.0 and attempts < maxtries:
        (traj, fraction) = group.compute_cartesian_path(way_points, step_len, 0.0, True)
        attempts += 1
        # 打印规划进程
        if attempts % 10 == 0:
            rospy.loginfo('still trying after  ' + str(attempts) + '  attempts...')
        if fraction == 1.0:
            return traj
            rospy.loginfo('Path computed successfully. Moving the arm.')

'''
## 寄存器操作函数群
'''
def write_joints(start_reg_addr,point, indexes):
    # 向串口下发目标关节值，并返回是否成功
    desired = [point.positions[i] for i in indexes]
    if DEBUG:
        current_joints = read_joints(POINT1_START_REG, 6)
        if current_joints:
            G_JOINT_MSG.header.stamp = rospy.Time.now()
            G_JOINT_MSG.position = current_joints
    for i in indexes:
        ## 循环发送各关节命令至机械手，运动到当前点
        # self.joints[i].set_current_psition(desired[i])
        # 把角度偏移DEGREE_OFFSET 并放大 DEGREE_RATIO倍，写入寄存器
        # 写入不成功,返回False
        if not write_reg(start_reg_addr,int((degrees(desired[i])+DEGREE_OFFSET)*DEGREE_RATIO)):
            return False
        # 寄存器地址+1，准备写入下一个关节值
        start_reg_addr += 1
    return True

def read_joints(start_reg_addr,joints_num):
    ## 读各关节值并返回
    joints = read_reg(start_reg_addr,joints_num)
    if joints == None:
        return None
    else:
        for i in range(len(joints)):
            joints[i] = radians(joints[i]/10.0-DEGREE_OFFSET)
        # print([degrees(i) for i in joints])
        return joints

def write_reg(addr, value):
    ## 写寄存器，并返回是否成功
    try:
        SER_ARM.write_register(addr,value)
        return True
    except:
        return False

def read_reg(addr,count = None):
    ## 读寄存器，成功则返回读取的值，否则返回None
    try:
        if count == None:
            return SER_ARM.read_register(addr)
        else:
            return SER_ARM.read_registers(addr,count)
    except:
        return None


def write_coil(addr, value):
    ## 写寄存器，并返回是否成功
    try:
        SER_ARM.write_bit(addr,value)
        return True
    except:
        return False


def decrease_traj_len(traj, step):
    ## 按步长减少路径长度方法
    l_temp = []  # 临时变量，取目标路径用
    len_traj = len(traj.points) # 路径长度
    l_index = range(0, len_traj, step)  # 取出需要留下的路径点下标
    print l_index
    # 若最后一个没取到，则加入
    if l_index[-1] < len_traj -1:
        l_index.append(len_traj-1)
    # 若最后一个和倒数第二个很接近，则删除
    if len(l_index) > 3 and l_index[-1] - l_index[-2] < step/2:
        del l_index[-2]
    # 取出目标路径点
    print l_index
    for i in l_index:
        l_temp.append(traj.points[i])
    # 目标路径点传入轨迹
    traj.points = l_temp
    print 'decreased length is : '+ str(len(traj.points))



def kill_process(port = 50008):
    ret = os.popen('netstat -nao|findstr' + str(port))
    str_list = ret.read().decode('gbk')
    ret_list = re.split('', str_list)
    try:
        process_pid = list(ret_list[0].split())[-1]
        os.popen('taskkill /pid ' + str(process_pid) + ' /F')
    except:
        return True
def server_main():
    ## socket线程主循环，接收来自视觉的目标请求命令
    global GOAL_POSITION
    global GOAL_FLAG

    host = '0.0.0.0'
    port = 50008
    kill_process()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(10)
    while not rospy.is_shutdown():
        # 等待CLIENT连接
        try:
            print('Waitting for socket client')
            conn, addr = s.accept()
            if addr:
                break
        except:
            pass
    goal_err = False

    while not GOAL_FLAG == 0:
        ## 等待机械手初始化完成
        print('Waitting for arm init')
        print(str(GOAL_FLAG))
        time.sleep(2)
    while not rospy.is_shutdown():
        ## 主循环
        try:
            ## 接收并GOAL_POSITION = [0.0, -0.99398, 0.18418] # 梨目标所在位置
            data = conn.recv(1024)
            print(data)
            s_head = data.rfind('*')
            s_end = data.rfind('#')
            if s_end > s_head:
                data = data[s_head+1:s_end]
                data = data.split(',')
            else:
                data = None

            if data:
                print(data)
                for i in range(len(data)):
                    ## data转换成float类型
                    try:
                        data[i] = float(data[i])
                    except:
                        goal_err = True
                        break
                if not goal_err:
                    # 返回收到合法data
                    conn.sendall('*received#')
                    GOAL_POSITION = copy.deepcopy(data)
                    GOAL_FLAG = 1
                    while GOAL_FLAG == 1:
                        # 等待机械手执行目标结果
                        time.sleep(2)
                    if GOAL_FLAG == 2:
                        # 返回目标抓取结果
                        conn.sendall('*get_goal#')
                    elif GOAL_FLAG == 3:
                        conn.sendall('*failed#')
                else:
                    # 返回受到不合法命令
                    conn.sendall('*err#')
                    goal_err = False
        except:
            ## 如果出错重新监听
            kill_process()
            print 'err'
            try:
                s = None
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.bind((host, port))
                s.listen(1)
                conn, addr = s.accept()
                goal_err = True
            except:
                kill_process()




# 执行轨迹
def execute_trajectory(traj):
    rospy.loginfo('Executing trajectory')
    rospy.logdebug(traj)
    
    # 取出轨迹
    try:
        indexes = [traj.joint_names.index(joint)
                    for joint in G_JOINTS]
    except ValueError as val:
        rospy.logerr('Invalid joint in trajetory.')

    len_trajs = len(traj.points)
    odd_flag = len_trajs % 2
    G_SER_IN_USE = 1
    i = 0
    while i <len_trajs:
        s_time = rospy.Time.now()
        timeout = 0
        read1_flag = read_reg(POINT1_OK_REG)
        read2_flag = read_reg(POINT1_OK_REG)
        # print str(read1_flag) + '========' + str(read2_flag)
        # 调试模式
        if DEBUG:
            read1_flag = not read1_flag
            read2_flag = not read2_flag
            current_position_reg = POINT1_START_REG
        else:
            current_position_reg = POINT_CUR_REG
        while read1_flag and read2_flag:
            ## 等待点1和点2中一个寄存器准备好接收关节值
            timeout +=0.01
            rospy.sleep(0.01)
            # 读机器人当前位姿POINT_CUR_REG
            current_joints = read_joints(current_position_reg, 6)
            if current_joints:
                G_JOINT_MSG.header.stamp = rospy.Time.now()
                G_JOINT_MSG.position = current_joints
            if timeout > 30 or read1_flag == None or read2_flag == None:
                return False
            read1_flag = read_reg(POINT1_OK_REG)
            read2_flag = read_reg(POINT2_OK_REG)
            if DEBUG:
                read1_flag = not read1_flag
                read2_flag = not read2_flag

        point = traj.points[i]
        # 轨迹点长度为奇数，且是最后一个
        if  not read1_flag:
            # print 'write point1=========='
            write_joints(POINT1_START_REG, point, indexes)
            write_reg(POINT1_OK_REG,1)
            i+=1
        elif (not read2_flag) and i < len_trajs:
            # print 'write point2========='
            point = traj.points[i]
            write_joints(POINT2_START_REG, point, indexes)
            write_reg(POINT2_OK_REG,1)
            i+=1
        e_time = rospy.Time.now()
        # print 'whole loop time is :' + str(e_time - s_time)
    G_SER_IN_USE = 0
    return True




class FollowController:
# 驱动核心代码
    def __init__(self,name='arm_controller'):
        
        self.name = name
        # action server 创建
        self.server = actionlib.SimpleActionServer(
                'r_rob_mover/follow_joint_trajectory', 
                FollowJointTrajectoryAction, 
                execute_cb = self.action_callback, auto_start = False)
        self.server.start()
        rospy.loginfo('Started FollowController')

    # 回调函数
    def action_callback(self, goal):
        print('********actionCb*********')
        rospy.loginfo(self.name + ' Action goal recieved.')
        traj = goal.trajectory
        rospy.loginfo(self.name + ':Done')

    

if __name__ == '__main__':
    try:
        rospy.loginfo('start follow controller...')
        arm_init()
    except rospy.ROSInterruptException:
        rospy.loginfo('Failed to start follow controller...')
    
