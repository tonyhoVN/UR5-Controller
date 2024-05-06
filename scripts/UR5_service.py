#!/usr/bin/env python
import rospy 
import tf 
from assignment_ur.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib import SimpleActionClient
import math

HOME = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0] # Home position of robot
JOINT_NAME = ["shoulder_pan_joint", 
              "shoulder_lift_joint", 
              "elbow_joint", 
              "wrist_1_joint",
              "wrist_2_joint",
              "wrist_3_joint"]

class UR5Service:
    def __init__(self):
        rospy.init_node('UR5_custom_service')
        self.get_state_service = rospy.Service('get_state', GetState, self.get_state)

    def get_state(request):
        response = GetStateResponse()
        # wait for joints_state message
        try:
            msg = rospy.wait_for_message("/joints_state", JointState, 1)
            response.state = msg
            return response
        except:
            rospy.logerr("Topic /joints_state has not been start")
            return

if __name__ == '__main__':
    ur5_service = UR5Service()
    try:
        rospy.loginfo("Start UR5 custom service")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

