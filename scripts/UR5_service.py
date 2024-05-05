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
        
        # Action client for joint controller
        self.pub = rospy.Publisher("/eff_joint_traj_controller/command", JointTrajectory, queue_size=1)

        self.start_robot()

    def start_robot(self):
        # Publish the goal point 
        while(True):
            try:
                rospy.wait_for_message("/joint_states", JointState, 5.0)
                target_point = JointTrajectoryPoint(positions=HOME, velocities=[0]*6, time_from_start=3.0)
                goal_msg = JointTrajectory()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.joint_names = JOINT_NAME
                goal_msg.points.append(target_point)
                self.pub.publish(goal_msg)
            except:
                rospy.logerr("CANNOT INITIAL ROBOT")
                pass

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

