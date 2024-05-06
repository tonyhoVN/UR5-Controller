#!/usr/bin/env python

from cgitb import reset
from ntpath import join
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from assignment_ur.srv import *


def main():
    # Create controller object
    client = rospy.ServiceProxy('/ur5_custom_service/get_robot_state', GetState)
    srv = GetStateRequest()
    result = client.call(srv)
    
    if result:
        print(result.state.name)
    else:
        print("not")

if __name__ == '__main__':
    main()