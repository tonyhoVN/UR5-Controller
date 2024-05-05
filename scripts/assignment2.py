#!/usr/bin/env python

from ntpath import join
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
import math
import moveit_planners_ompl

class UR5Controller:
    def __init__(self):
        rospy.init_node('ur5_joint_controller')
        # client 
        pass


def main():
    # Create controller object
    ur5_joint_controller = UR5Controller()   
    try:
        rospy.spin()        

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()