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



def set_goal_loop():
    ## 设置目标位置进程主循环
    # 关键点定义
    print 'start set_goal'
    home_pose =[radians(i) for i in [89.156,57.127,38.238,-6.354,96.253,0]]
    release_pose =[radians(i) for i in [89.154,50.331,13.980,-6.95,65.4,0]]
    pose1 = [radians(i) for i in [-19.654,88.135,-5.039,4.160,78.831,0]]
    work_pose_home = [radians(i) for i in [-19.655,75.795,-24.434,5.566,-33.123,0]]

    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_cartesian_demo', anonymous=True)
    # 初始化move group控制机械臂中的 arm
    arm = moveit_commander.MoveGroupCommander('arm')
    # 获取终端link名称
    end_effector_link = arm.get_end_effector_link()
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    # 设置位姿的允许误差
    arm.set_goal_position_tolerance(0.003)
    arm.set_goal_orientation_tolerance(0.005) 
    ''' 

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
    # 设置机械臂当前的状态为运动初始状态
    arm.set_start_state_to_current_state()
# 清空位姿信息
    # arm.clear_pose_targets()
    # 设置机械臂终端运动的目标位姿
    # arm.set_pose_target(target_pose, end_effector_link)i
    # 设置机械臂终端目标关节值
    arm.set_joint_value_target(work_pose_home)
    
    '''
    # 笛卡尔路径规划
    waypoints=[work_pose_home, pose1, home_pose, release_pose, home_pose, pose1, work_pose_home]
    fraction = 0.0 # 路径规划覆盖率
    maxtries = 100 # 最大尝试规划次数
    attempts = 0 # 已经尝试规划次数
    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路径点
    while fraction < 1.0 and attempts < maxtries:
        (traj, fraction) = arm.compute_cartesian_path(waypoints, 0.01,0.0,True)
        attempts += 1
        # 打印规划进程
        if attempts % 10 == 0:
            rospy.loginfo('still trying after' + str(attempts)) + 'attempts...'
        if fraction == 1.0:
            rospy.loginfo('Path computed successfully. Moving the arm.')
    '''
    # 规划运动路径
    # traj = arm.plan()
    # 执行运动路径
    print traj
    rospy.sleep(1)

if __name__ == '__main__':
    set_goal_loop()
