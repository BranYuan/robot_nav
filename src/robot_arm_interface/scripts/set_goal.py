#!/usr/bin/env python
# __*__ coding:utf-8 __*__
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
'''
## 四元树转欧拉角函数，形参：(quaternion, axes = 'syxz')
#  axes为对应欧拉旋转顺序，quaternion为四元数list

## 欧拉角转四元数，形参(ai,aj,ak,axes='sxyz')，
#  ai,aj,ak: 欧拉角的roll pitch yaw, 即机器人的x y z;axes轴的24种排列

##ros中使用时,两函数值都应为axes='sxyz'

'''
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():    
    ## 初始化moveit_commander和node
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

    ## 实例化一个robot，该robot为与moveit交互的接口
    #robot = moveit_commander.RobotCommander()

    ## robot与外界交互的实例
    #scene = moveit_commander.PlanningSceneInterface()

    ## 与move group joints 交互的实例，用于规划与执行轨迹
    group = moveit_commander.MoveGroupCommander("arm")


    ## DisplayTrajectory publisher，发布轨迹显示到RVIZ用于可视化
    #display_trajectory_publisher = rospy.Publisher(
    #                                  '/move_group/display_planned_path',
    #                                  moveit_msgs.msg.DisplayTrajectory)

    ## 等待RVIZ初始化
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    print "============ Starting tutorial "

    ## 获取基本信息
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## 获取参考坐标系信息
    #print "============ Reference frame: %s" % group.get_planning_frame()

    ## 获取该组的末端执行器信息 
    #print "============ Reference frame: %s" % group.get_end_effector_link()

    ##获取机器人所有规划组信息 
    #print "============ Robot Groups:"
    #print robot.get_group_names()

    ## 获取机器人状态信息 
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print "============"


    ## 基于姿态的轨迹规划

    ## 设置目标位姿信息
    '''
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = 0.7758
    pose_target.orientation.y = 0.1637
    pose_target.orientation.z = -0.6027
    pose_target.orientation.w = 0.0897
    pose_target.position.x = -1.322
    pose_target.position.y = -0.5466
    pose_target.position.z = 0.1935
    group.set_pose_target(pose_target)

    ## 轨迹规划请求,并在RVIZ显示规划的轨迹
    # plan1 = group.plan()
    print (group.go())
    '''

    '''
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(2)


    ## 显示规划完成的轨迹，与group.plan()重复
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);
    group.go()
    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(2)
    '''

    ## 清空刚设置的位姿信息
    group.clear_pose_targets()

    ## 获取当前group的关节位置信息 
    group_variable_values = group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values

    ## 修改其中一个关节信息，进行轨迹规划并可视化轨迹
    group_variable_values[0] = 1.0
    group.set_joint_value_target(group_variable_values)

    #group.go()
    print group.plan()
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(2)

    '''
    ## 笛卡尔路径
    ## ^^^^^^^^^^^^^^^
    ## 一系列路径点的笛卡尔路径规划 
    waypoints = []

    # 以但前位姿为开始位姿
    waypoints.append(group.get_current_pose().pose)

    # 1.夹抓原点前移(+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x + 0.1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # 2.下移(-z)
    wpose.position.z -= 0.30
    waypoints.append(copy.deepcopy(wpose))

    # 3.侧移(+y)
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    ## 希望在笛卡尔路径上的以1cm的距离插入重新规划的点
    ## 故在笛卡尔变换中设置eef_step = 0.01
    ## jump_thresholed = 0.0,以实现有效的取消路径
    (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
    group.go()
    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(2)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
    '''


if __name__=='__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass

