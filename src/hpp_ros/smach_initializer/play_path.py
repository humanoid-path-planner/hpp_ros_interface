import smach
import smach_ros
from std_msgs.msg import UInt32, Empty, String
from hpp_ros_interface.srv import *
import std_srvs.srv
import ros_tools

_outcomes = ("succeeded", "aborted", "preempted")

class InitializePath(smach.State):
    hppTargetPubDict = {
            "read_path": [ UInt32, 1 ],
            }
    hppTargetSrvDict = {
            "reset_topics": [ std_srvs.srv.Empty, ],
            "add_center_of_mass": [ SetString, ],
            "add_operational_frame": [ SetString, ],
            "add_center_of_mass_velocity": [ SetString, ],
            "add_operational_frame_velocity": [ SetString, ],
            }

    def __init__(self):
        super(InitializePath, self).__init__(self, outcomes=_outcomes)

        self.targetSrv = ros_tools.createServices (self, "/hpp/target", self.hppTargetSrvDict, serve=False)
        self.targetPub = ros_tools.createTopics (self, "/hpp/target", self.hppTargetPubDict, subscribe=False)

    def execute (self, userdata):
        # TODO makeSot
        self.targetSrv["reset_topics"](std_srvs.srv.Empty())
        # # TODO Set the topics specifying the targets
        # self.targetSrv["add_center_of_mass"](SetStringRequest("name"))
        # self.targetSrv["add_center_of_mass_velocity"](SetStringRequest("name"))
        # # ...

        self.targetPub["read_path"].publish (UInt32 (userdata.pathId))
        return _outcomes[0]

class PlayPath (smach.State):
    hppTargetPubDict = {
            "publish": [ Empty, 1 ],
            }
    subscribersDict = {
            "sot_hpp": {
                "error": [ String, "handleError" ],
                "interrupt": [ String, "handleInterrupt" ],
                }
            "hpp" : {
                "target": {
                    "publish_done": [ Empty, "handleFinished" ]
                    }
                }
            }

    def __init__(self):
        super(PlayPath, self).__init__(self, outcomes=["succeeded"])

        self.targetPub = ros_tools.createTopics (self, "/hpp/target", self.hppTargetPubDict, subscribe=False)
        self.subscribers = self._createTopics ("/sot_hpp", self.subscribersDict, subscribe=True)

        self.error = None
        self.interruption = None

    def handleError (self, msg):
        self.error = msg.data

    def handleInterrupt (self, msg):
        self.interruption = msg.data
        rospy.loginfo(str(msg.data))
        self.done = True

    def handleFinished (self, msg):
        self.done = True

    def execute(self, userdata):
        # TODO Check that there the current SOT and the future SOT are compatible ?
        # TODO switch from old SOT to new SOT
        self.done = False
        self.targetPub["publish"](Empty())
        # Wait for errors or publish done
        rate = rospy.Rate (1000)
        userdata.error = None
        userdata.interruption = None
        while not self.done:
            if self.error is not None:
                # TODO handle error
                rospy.logerror(str(self.error))
                userdata.error = self.error
                return _outcomes[1]
            rate.sleep()
        if self.interruption is not None:
            userdata.interruption = self.interruption
            return _outcomes[2]
        return _outcomes[0]

class ErrorHandling (smach.State):
    subscribersDict = {
            "sot_hpp": {
                "error": [ String, "handleError" ],
                }
            "hpp" : {
                "target": {
                    "publish_done": [ Empty, "handleFinished" ]
                    }
                }
            }

    def __init__(self):
        super(PlayPath, self).__init__(self, outcomes=["succeeded"])

        self.targetPub = ros_tools.createTopics (self, "/hpp/target", self.hppTargetPubDict, subscribe=False)
        self.subscribers = self._createTopics ("/sot_hpp", self.subscribersDict, subscribe=True)

        self.error = None

    def handleError (self, msg):
        self.error = msg

    def handleFinished (self, msg):
        self.done = True

    def execute(self, userdata):
        # TODO Check that there the current SOT and the future SOT are compatible ?
        # TODO switch from old SOT to new SOT
        self.done = False
        self.targetPub["publish"](Empty())
        # Wait for errors or publish done
        rate = rospy.Rate (1000)
        while not self.done:
            if self.error is not None:
                # TODO handle error
                rospy.logerror(str(self.error))
                return _outcomes[1]
            rate.sleep()
        return _outcomes[0]
