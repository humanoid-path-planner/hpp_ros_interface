#!/usr/bin/env python
from hpp.ros_interface.smach_initializer import makeStateMachine
import rospy

rospy.init_node ('sm_sot_hpp')
sm, sis = makeStateMachine()

sis.start()
outcome = sm.execute()
sis.stop()
