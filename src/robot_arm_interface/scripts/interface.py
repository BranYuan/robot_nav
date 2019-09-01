#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItEfDemo():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_redon_action', anonymous=True)
    arm = moveit_commander.MoveGroupCommander('arm')

    arm.set_goal_joint_tolerance(0.001)

    joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
    
    arm.go()
    rospy.sleep(1)
    
    joint_positions = [-1.2, -1.274, 0.832, 0.0820, -1.273, -0.003]
    arm.go()
    rospy.sleep(1)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass

    

