#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import kermit.controller.cubic_spline_planner as cubic_spline_planner
from kermit.controller.stanley_control_ros import *

if __name__ == '__main__':
    rospy.init_node("Node")
    main(None, None)
