from hpp_ros_interface.smach_initializer import makeStateMachine
import rospy

rospy.init_node ('sm_sot_hpp')
sm, sis = makeStateMachine()

sis.start()
outcome = sm.execute()
rospy.spin()
sis.stop()
