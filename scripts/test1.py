#!/usr/bin/env python

import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.join(current_dir, "../")
sys.path.append(current_dir)
sys.path.append(parent_dir)

import UR5_API

m = UR5_API.get_joint_state()
print(m)