#!/usr/bin/env python
import hpp_ros_interface.planning_request_adapter as pra
import hpp_ros_interface.hpp_server_initializer as hsi
import rospy

_pra = pra.PlanningRequestAdapter("/joint_states")
_hsi = hsi.HppServerInitializer()

rospy.init_node('hpp_server_connection')
rospy.spin()
