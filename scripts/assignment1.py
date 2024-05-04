#!/usr/bin/env python
from ntpath import join
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
import math

HOME = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0] # Home position of robot

class UR5JointController:
    def __init__(self):
        rospy.init_node('ur5_joint_controller')
        
        # Action client for joint controller
        self.client = SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Subscriber and publisher 
        self.sub = rospy.Subscriber("/joint_states", JointState, self.sine_wave_move)
        self.pub = rospy.Publisher("/eff_joint_traj_controller/command", JointTrajectory, queue_size=1)
        
        # Duration
        self.hz = 60
        self.duration  = rospy.Duration(float(1.0/self.hz))
        self.omega = math.radians(90) # angular velocity 90 degree/s
        self.amplitude = math.radians(30) # amplitude of 30 degrees
    
    def sine_wave_move(self, joint_msg):
        # Get current timestamp
        current_time = joint_msg.header.stamp.to_sec()

        # Create goal message 
        target_joints = [j + self.amplitude*math.sin(self.omega*current_time) for j in HOME]
        target_point = JointTrajectoryPoint(positions=target_joints, velocities=[0]*6, time_from_start=self.duration)
        goal_msg = JointTrajectory()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.joint_names = joint_msg.name
        goal_msg.points.append(target_point)

        # Publish the goal point 
        self.pub.publish(goal_msg)

def main():
    # Create controller object
    ur5_joint_controller = UR5JointController()   
    try:
        rospy.spin()        

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
