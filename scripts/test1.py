#!/usr/bin/env python

import os, sys
# current_dir = os.path.dirname(os.path.abspath(__file__))
# parent_dir = os.path.join(current_dir, "../")
# sys.path.append(current_dir)
# sys.path.append(parent_dir)
from assignment_ur.srv import GetState

import rospy 
from UR5_API import ur5


print(ur5.get_joint_name())


    # listener = tf.TransformListener()

    # # Wait for the first transforms to become available
    # listener.waitForTransform('/base_link', '/wrist_3_link', rospy.Time().now(), rospy.Duration(4.0))

    # # Get the transform between the two frames
    # try:
    #     (trans, rot) = listener.lookupTransform('/base_link', '/end_effector', rospy.Time(0))
    #     print("Translation:", trans)
    #     print("Rotation:", rot)
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     rospy.logerr("Failed to get transform")
