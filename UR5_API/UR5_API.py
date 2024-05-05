#!/usr/bin/env python
__ROS__ = True
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import rospy

def get_joint_state():
    msg = rospy.wait_for_message("/joint_states",JointState,3)
    return msg.name
