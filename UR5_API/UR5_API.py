#!/usr/bin/env python
__ROS__ = True
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import rospy
import tf
from assignment_ur.srv import *

def get_joint_state():
    """
    Get state of robot's joints
    return: JointState
    
    Class JointState has 4 attributes 
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    """
    # Create controller object
    client = rospy.ServiceProxy('/ur5_custom_service/get_robot_state', GetState)
    srv = GetStateRequest()
    result = client.call(srv)
    if result:
        return result.state
    else:
        raise("Cannot get joint state")


def get_posj():
    """
    Get the joints position of robot
    """
    state = get_joint_state()
    if state == None:
        return []
    else:
        posj = state.position
    return posj

def get_posx():
    """
    Get position of end-effector frame w.r.t base frame of robot 
    """
    posx = []
    try:
        listener = tf.TransformListener()
        (trans, rot) = listener.lookupTransform('base_link', 'wrist_3_link', rospy.Time(0))
        posx = [trans,rot]
    except:
        print("Not find transform")

    return posx