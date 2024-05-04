#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
import math


joint_names = ['shoulder_pan_joint', 
               'shoulder_lift_joint', 
               'elbow_joint',
               'wrist_1_joint', 
               'wrist_2_joint', 
               'wrist_3_joint']

class UR5JointController:
    def __init__(self):
        rospy.init_node('ur5_joint_controller')
        
        # Action client for joint controller
        self.client = SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
    
    def sine_wave_move(self):
        current_joint = rospy.wait_for_message("/joint_state", JointState)
        rospy.Time.now()
        pass

    def control_joint_positions(self, joint_positions):
        # Create FollowJointTrajectoryGoal message
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint(positions=joint_positions, velocities=[0]*6, time_from_start=rospy.Duration(5))
        
        # Add trajectory point to the goal
        goal.trajectory.points.append(point)

        # Send goal to the action server
        self.client.wait_for_server()
        self.client.send_goal(goal)

if __name__ == '__main__':
    
    # Create controller object
    ur5_joint_controller = UR5JointController()
    rate = rospy.Rate(10)  # 10 Hz    
    try:
        joint_positions = [0.0, -math.pi/2, 0.0, -math.pi/3, 0.0, 0.0]
        ur5_joint_controller.control_joint_positions(joint_positions)            
        # while not rospy.is_shutdown():
        #     # Publish joint positions
        #     rate.sleep()

    except rospy.ROSInterruptException:
        ur5_joint_controller.client.cancel_all_goals()
        pass