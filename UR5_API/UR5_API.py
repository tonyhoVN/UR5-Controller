#!/usr/bin/env python
__ROS__ = True
# from geometry_msgs.msg import Point
# from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from sensor_msgs.msg import JointState
import rospy
from assignment_ur.srv import *

# Common Client 
get_state_client_ = rospy.ServiceProxy('/ur5_custom_service/get_robot_state', GetState)
move_home_client_ = rospy.ServiceProxy('/ur5_custom_service/move_home', MoveHome)
move_jnt_space_client_ = rospy.ServiceProxy('/ur5_custom_service/move_joint_space', JointSpaceMotion)
move_cart_space_client_ = rospy.ServiceProxy('/ur5_custom_service/move_cartesian_space', CartesianSpaceMotion)

# Get state client
def get_joint_name():
    """
    Get name of robot's joints
    """
    srv = get_state_client_()
    if srv:
        return list(srv.joints_name)
    else:
        raise("Cannot get joint name")

def get_posj():
    """
    Get the joints position of robot
    """
    srv = get_state_client_()
    if srv:
        return list(srv.posj)
    else:
        raise("Cannot get joint state")

def get_posx():
    """
    Get position of end-effector frame w.r.t base frame of robot 
    """
    srv = get_state_client_()
    if srv:
        return list(srv.posx)
    else:
        raise("Cannot get joint state")

def move_home():
    """
    Move robot to home position
    """
    srv = move_home_client_()
    if srv:
        print("Robot moved to home position")
    else:
        raise("Cannot move to home")

def move_joint_space(joints1, joints2, joint_vel_max, joint_acc_max):

    msg = JointSpaceMotionRequest()
    msg.joints1 = joints1
    msg.joints2 = joints2
    msg.joints_vel = joint_vel_max
    msg.joints_acc = joint_acc_max

    result = move_jnt_space_client_.call(msg)

    if result.result:
        print("Move success")
    else:
        raise("Cannot make the movement")

def move_cartesian_space(point1, point2, linear_vel_max, linear_acc_max):
    msg = CartesianSpaceMotionRequest()
    msg.point1 = point1
    msg.point2 = point2
    msg.linear_vel = linear_vel_max
    msg.linear_acc = linear_acc_max
    result = move_cart_space_client_.call(msg)

    if result.result:
        print("Move success")
    else:
        raise("Cannot make the movement")



move_home()
move_cartesian_space([0.1,0,0.5], [0.1,0,0.3], 0.2, 0.2)