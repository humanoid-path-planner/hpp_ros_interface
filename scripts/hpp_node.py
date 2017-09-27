#!/usr/bin/env python
import hpp_ros_interface.planning_request_adapter as pra
import rospy

node = pra.PlanningRequestAdapter("/joint_states")
pra.init_node()
rospy.spin()
