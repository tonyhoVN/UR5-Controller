#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib import SimpleActionClient

class UR5JointController:
    def __init__(self):
        rospy.init_node('ur5_joint_controller')
        
        # Define action client for FollowJointTrajectory action
        self.client = SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def control_joint_positions(self, joint_positions):
        # Create FollowJointTrajectoryGoal message
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(2)  # 1 second
        
        # Add trajectory point to the goal
        goal.trajectory.points.append(point)

        # Send goal to the action server
        self.client.wait_for_server()
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    try:
        ur5_joint_controller = UR5JointController()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Example joint positions (in radians)
            joint_positions = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
            
            # Publish joint positions
            ur5_joint_controller.control_joint_positions(joint_positions)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass