#!/usr/bin/env python
import UR5_API

print(UR5_API.get_joint_name())
UR5_API.move_home()
UR5_API.move_cartesian_space(point2=[400,100,100], linear_acc_max=5)
