#!/usr/bin/env python
import rospy, roslib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
# import std_msgs.msg
import actionlib

JOINT_NAMES = ['shoulder_pan_joint', 
                'shoulder_lift_joint', 
                'elbow_joint',
                'wrist_1_joint', 
                'wrist_2_joint', 
                'wrist_3_join']

Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]

class WaveSignal:
    def __init__(self):
        rospy.init_node("wave_signal_node")
        self.client = actionlib.SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', 
                                                    FollowJointTrajectoryAction)
        

    def setupMessage(self):
        self.cmd_message.trajectory.joint_names.clear()
    
    def command(self):
        # Get current joint position 
        joint_state = rospy.wait_for_message("/joint_states", JointState)
        
        # Setup message 
        cmd_message = FollowJointTrajectoryGoal()
        cmd_message.trajectory = JointTrajectory()
        cmd_message.trajectory.joint_names = joint_state.name
        cmd_message.trajectory.points = [
            JointTrajectoryPoint(positions=joint_state.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(5.0))
        ]

        # Set header 
        cmd_message.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        # send message
        self.client.wait_for_server()
        self.client.send_goal(cmd_message)
        self.client.wait_for_result()
        print("finish")

        return self.client.get_result()

    def move_repeated(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            d = 2.0
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            for i in range(30):
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 1
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 1
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 2
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

def main():
    wave_cmd = WaveSignal()
    rate = rospy.Rate(10)
    try:
        result = wave_cmd.command()
        while not rospy.is_shutdown():
            wave_cmd.move_repeated()
            rate.sleep()

        print("STOP")
        wave_cmd.client.cancel_goal()

    except KeyboardInterrupt:
        print("STOP")
        wave_cmd.client.cancel_goal()
        raise


if __name__ == '__main__':
    main()