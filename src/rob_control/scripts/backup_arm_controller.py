#!/usr/bin/env python
# __*__coding:utf-8__*__

import sys
import rospy
import actionlib
import threading
import moveit_commander
import moveit_msgs.msg
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

DEGREE_OFFSET = 400 # 机器人寄存器不能写入负数，故角度偏移400
DEGREE_RATIO = 10        # 角度放大比例

#寄存器地址
ARM_READY_REG = 100
POINT1_OK_REG = 101
POINT2_OK_REG = 102
POINT_OK_REG = 106
POINT_CUR_REG = 107
POINT1_START_REG = 113
POINT2_START_REG = 119



def arm_init():
    global G_SER_IN_USE
    global G_JOINT_MSG
    global SER_ARM
    
    rospy.init_node('arm_controller')
    G_JOINT_MSG.header = Header()
    G_JOINT_MSG.header.stamp = rospy.Time.now()
    G_JOINT_MSG.name = G_JOINTS
    G_JOINT_MSG.position = G_POSITION
    G_JOINT_MSG.velocity = []
    G_JOINT_MSG.effort = []
    SER_ARM.serial.baudrate = 115200
    SER_ARM.serial.timeout = 0.1
    SER_ARM.serial.write_timeout = 0.1
    
    rate = rospy.Rate(50.0)
    pup = rospy.Publisher('joint_states', JointState, queue_size = 5)
    # sub = rospy.Subscriber('joint_states', JointState, callback)
   
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
    
    #定义并启动发布者多线程
    pup_thread = threading.Thread(target=pup_thread_loop,args = (pup,), name='PubLoopThread')
    # 控制器线程
    act_server_thread = threading.Thread(target=act_server_thread_loop, name='ActLoopThread') 
    # 设置目标位置线程
    set_goal_thread = threading.Thread(target=set_goal_loop, name='SetGoalThread') 
    
    pup_thread.start()
    act_server_thread.start()
    set_goal_thread.start()
    print 'start thread'
    pup_thread.join()
    act_server_thread.join()
    set_goal_thread.joint()
    # rospy.spin()

# 发布者线程主循环
def pup_thread_loop(pup):
    # pup = rospy.Publisher('joint_states', JointState, queue_size = 5)
    while not rospy.is_shutdown():
        if not G_SER_IN_USE:
            G_JOINT_MSG.header.stamp = rospy.Time.now()
        pup.publish(G_JOINT_MSG)
        # print G_JOINT_MSG
        rospy.sleep(0.1)
def act_server_thread_loop():
    FollowController()
    rospy.spin()


def set_goal_loop():
    pass
    ## 设置目标位置进程主循环
    # 关键点定义
    print 'start set_goal'
    home_pose =[radians(i) for i in [89.156,57.127,38.238,-6.354,96.253,0]]
    release_pose =[radians(i) for i in [89.154,50.331,13.980,-6.95,65.4,0]]
    pose1 = [radians(i) for i in [-19.654,
    work_pose_home =[radians(i) for i in [-19.655,75.795,-24.434,5.566,-33.123]]
    
    rospy.sleep(10)

    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化move group控制机械臂中的 arm
    arm = moveit_commander.MoveGroupCommander('arm')
    # 获取终端link名称
    end_effector_link = arm.get_end_effector_link()
    print end_effector_link
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    # 设置位姿的允许误差
    arm.set_goal_position_tolerance(0.003)
    arm.set_goal_orientation_tolerance(0.005)

    



    ## 设置机械臂工作空间中的目标位姿
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()

    target_pose.pose.position.x = 0.838948
    target_pose.pose.position.y = -0.07723
    target_pose.pose.position.z = -0.09544
    target_pose.pose.orientation.x = -0.4575
    target_pose.pose.orientation.y = 0.8274
    target_pose.pose.orientation.z = 0.21285
    target_pose.pose.orientation.w = 0.2461

    '''
    target_pose.pose.position.x = 0.191995
    target_pose.pose.position.y = 0.213868
    target_pose.pose.position.z = 0.520436
    target_pose.pose.orientation.x = 0.911822
    target_pose.pose.orientation.y = -0.0269758
    target_pose.pose.orientation.z = 0.285694
    target_pose.pose.orientation.w = -0.293653
    '''
    # 设置机械臂当前的状态为运动初始状态
    arm.set_start_state_to_current_state()
    # 设置机械臂终端运动的目标位姿
    arm.set_pose_target(target_pose, end_effector_link)
    # 规划运动路径
    traj = arm.plan()
    # 执行运动路径
    arm.execute(traj)
    rospy.sleep(1)
'''


    arm.set_goal_joint_tolerance(0.005)
    
    cur_pose = work_pose_home
    arm.set_joint_value_target(cur_pose)

    arm.go()
    rospy.sleep(1)
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.RobotCommander()

    group = moveit_commander.MoveGroupCommander('arm')
    

    while not rospy.is_shutdown():
        cur_pose = G_JOINT_MSG.position
        for i in range(len(work_pose_home)):
            ## 等待机器人回原点
            while abs(cur_pose[i]-work_pose_home[i]) > 0.005:
                print 'wait for arm go to work_pose_home'
                rospy.sleep(2)
        HOME_FLAG = True


        group.get_planning_frame()
        group.get_end_effector_link()
        arm.get_group_names()
        print arm.get_current_state()
        
        print 'go to first point'
        target_pose = pose1
        cur_pose = group.get_current_joint_values()
        group.set_joint_value_target(target_pose)
        group.go()
        cur_pose = G_JOINT_MSG.position
        rospy.sleep(50)
        for i in range(len(cur_pose)):
            while abs(target_pose[i]-cur_pose[i])>0.005:
                print 'wait for arm go to target_pose'
                rospy.sleep(0.5)




        print 'go to 2nd point'
        target_pose =home_pose
        cur_pose = group.get_current_joint_values()
        group.set_joint_value_target(target_pose)
        group.go()
        cur_pose = G_JOINT_MSG.position
        for i in range(len(cur_pose)):
            while abs(target_pose[i]-cur_pose[i])>0.005:
                print 'wait for arm go to target_pose'



        print 'go to 3rd point'
        target_pose = release_pose
        cur_pose = group.get_current_joint_values()
        group.set_joint_value_target(target_pose)
        group.go()
        cur_pose = G_JOINT_MSG.position
        for i in range(len(cur_pose)):
            while abs(target_pose[i]-cur_pose[i])>0.005:
                print 'wait for arm go to target_pose'
        


        print 'go to 2nd point'
        target_pose =home_pose
        cur_pose = group.get_current_joint_values()
        group.set_joint_value_target(target_pose)
        group.go()
        cur_pose = G_JOINT_MSG.position
        for i in range(len(cur_pose)):
            while abs(target_pose[i]-cur_pose[i])>0.005:
                print 'wait for arm go to target_pose'

        print 'go to first point'
        target_pose = pose1
        cur_pose = group.get_current_joint_values()
        group.set_joint_value_target(target_pose)
        group.go()
        cur_pose = G_JOINT_MSG.position
        for i in range(len(cur_pose)):
            while abs(target_pose[i]-cur_pose[i])>0.005:
                print 'wait for arm go to target_pose'



        print 'go to work_pose_home'
        target_pose = work_pose_home
        cur_pose = group.get_current_joint_values()
        group.set_joint_value_target(target_pose)
        group.go()
        cur_pose = G_JOINT_MSG.position
        for i in range(len(cur_pose)):
            while abs(target_pose[i]-cur_pose[i])>0.005:
                print 'wait for arm go to target_pose'
    '''

'''
## 寄存器操作函数群
'''
def write_joints(start_reg_addr,point, indexes):
    # 向串口下发目标关节值，并返回是否成功
    desired = [point.positions[i] for i in indexes]
    for j in indexes:
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





class FollowController:
# 驱动核心代码
    def __init__(self,name='arm_controller'):
        
        self.name = name

        # 初始化机械臂关节，并把关节放入joints列表中
        self.joints = G_JOINTS
        self.joint_state_msg = G_JOINT_MSG 
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

        # 判断轨迹的joint和要控制的joint是否一致
        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = 'Trajectory joint names does not match'
                    rospy.logerr(msg)
                    self.server.set_aborted(text = msg)
                    return
            rospy.logwarn('Extra joints in trajectory')
        
        # 判断目标轨迹是否有效
        if not traj.points:
            msg = 'Trajectory is empty'
            rospy.logerr(msg)
            self.server.set_aborted(text = msg)
            return

        # 取要执行轨迹的关节的下标
        try:
            indexes = [traj.joint_names.index(joint) 
                        for joint in self.joints]
            print 'Trajectory joint names indexes is'
            print indexes
        except ValueError as val:
            msg = 'Trajectory invalid.'
            rospy.logerr(msg)
            self.server.set_aborted(text = msg)
            return
        
        if self.executeTrajectory(traj):
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text = 'Execution failed.')

        rospy.loginfo(self.name + ':Done')

    
    # 执行轨迹
    def executeTrajectory(self, traj):
        rospy.loginfo('Executing trajectory')
        rospy.logdebug(traj)
        
        # 取出轨迹
        try:
            indexes = [traj.joint_names.index(joint)
                        for joint in self.joints]
        except ValueError as val:
            rospy.logerr('Invalid joint in trajetory.')

        
        # 获取开始时间戳，moveit默认使用0，需要填入
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        len_trajs = len(traj.points)
        odd_flag = len_trajs % 2


        G_SER_IN_USE = 1
        for i in range(0,len_trajs,2):
            timeout = 0
            read_flag = read_reg(POINT_OK_REG)
            print ('=============================' + str(read_flag))
            while read_flag:
                ## 等待寄存器准备好接收关节值
                timeout +=0.01
                rospy.sleep(0.01)
                # 读机器人当前位姿POINT_CUR_REG
                current_joints = read_joints(POINT1_START_REG, 6)
                self.joint_state_msg.header.stamp = rospy.Time.now()
                self.joint_state_msg.position = current_joints
                G_JOINT_MSG = self.joint_state_msg

                if timeout > 10 or read_flag == None:
                    return False
                read_flag = read_reg(POINT_OK_REG)
            point = traj.points[i]
            write_joints(POINT1_START_REG, point, indexes)
            # 轨迹点长度为奇数，且是最后一个
            if odd_flag and i == len_trajs-1:
                write_reg(POINT1_OK_REG,1)
                write_reg(POINT2_OK_REG,0)
                write_reg(POINT_OK_REG,1)
                break
            else:
                point = traj.points[i+1]
                write_joints(POINT2_START_REG, point, indexes)
                write_reg(POINT1_OK_REG,1)
                write_reg(POINT2_OK_REG,1)
                write_reg(POINT_OK_REG,1)
            '''
            while rospy.Time.now() + 2 * rospy.Duration(0.01) < start:
                print rospy.Time.now
                print start
                rospy.sleep(0.01)
            rospy.sleep(0.1)
            '''
        G_SER_IN_USE = 0
        return True

if __name__ == '__main__':
    try:
        rospy.loginfo('start follow controller...')
        arm_init()
    except rospy.ROSInterruptException:
        rospy.loginfo('Failed to start follow controller...')
    
