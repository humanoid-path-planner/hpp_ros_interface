#!/usr/bin/env python
import sys, rospy, hpp_ros_interface.planning_request_adapter as pra
if "hpp-manipulation-server" in sys.argv:
    import hpp_ros_interface.manipulation.hpp_server_initializer as hsi
    print "Launching manipulation client"
else:
    import hpp_ros_interface.hpp_server_initializer as hsi
    print "Launching default client"

_pra = pra.PlanningRequestAdapter("/joint_states")
_hsi = hsi.HppServerInitializer()

rospy.init_node('hpp_server_connection')
rospy.spin()
