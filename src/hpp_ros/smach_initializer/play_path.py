import smach, smach_ros, rospy
from std_msgs.msg import UInt32, Empty, String, Float64
from hpp_ros_interface.srv import *
from hpp_ros_interface.msg import *
from hpp_ros_interface.client import HppClient
import std_srvs.srv
from hpp_ros_interface import ros_tools

_outcomes = ["succeeded", "aborted", "preempted"]

class InitializePath(smach.State):
    hppTargetPubDict = {
            "read_path": [ UInt32, 1 ],
            "read_subpath": [ ReadSubPath, 1 ],
            }
    hppTargetSrvDict = {
            "reset_topics": [ std_srvs.srv.Empty, ],
            "add_center_of_mass": [ SetString, ],
            "add_operational_frame": [ SetString, ],
            "add_center_of_mass_velocity": [ SetString, ],
            "add_operational_frame_velocity": [ SetString, ],
            }

    def __init__(self):
        super(InitializePath, self).__init__(
                outcomes = _outcomes,
                input_keys = [ "pathId", "times", "currentSection" ],
                output_keys = [ "transitionId", "currentSection" ],
                )

        self.targetSrv = ros_tools.createServices (self, "/hpp/target", self.hppTargetSrvDict, serve=False)
        self.targetPub = ros_tools.createTopics (self, "/hpp/target", self.hppTargetPubDict, subscribe=False)
        self.hppclient = HppClient (False)

    def execute (self, userdata):
        userdata.currentSection += 1
        if userdata.currentSection + 1 >= len(userdata.times):
            return _outcomes[0]
        start = userdata.times[userdata.currentSection]
        length = userdata.times[userdata.currentSection + 1] - start

        hpp = self.hppclient._hpp()
        manip = self.hppclient._manip()
        print userdata.pathId, start + length / 2
        userdata.transitionId = manip.problem.edgeAtParam(userdata.pathId, start + length / 2)
        # userdata.transitionId = manip.problem.edgeAtParam(userdata.pathId, start)

        self.targetPub["read_subpath"].publish (ReadSubPath (userdata.pathId, start, length))
        print "Start reading subpath. Waiting for one second."
        rospy.sleep(1)
        return _outcomes[2]

class PlayPath (smach.State):
    hppTargetPubDict = {
            "publish": [ Empty, 1 ],
            }
    subscribersDict = {
            "sot_hpp": {
                "error": [ String, "handleError" ],
                "interrupt": [ String, "handleInterrupt" ],
                "control_norm_changed": [ Float64, "handleControlNormChanged" ],
                },
            "hpp" : {
                "target": {
                    "publish_done": [ Empty, "handleFinished" ]
                    }
                }
            }
    serviceProxiesDict = {
            'sot': {
                'plug_sot': [ PlugSot, ]
                }
            }

    def __init__(self):
        super(PlayPath, self).__init__(
                outcomes = _outcomes,
                input_keys = [ "transitionId", ],
                output_keys = [ ])

        self.targetPub = ros_tools.createTopics (self, "/hpp/target", self.hppTargetPubDict, subscribe=False)
        self.subscribers = ros_tools.createTopics (self, "", self.subscribersDict, subscribe=True)
        self.serviceProxies = ros_tools.createServices (self, "", PlayPath.serviceProxiesDict, serve=False)

        self.done = False
        self.error = None
        self.interruption = None
        self.control_norm_ok = False

    def handleError (self, msg):
        self.error = msg.data

    def handleInterrupt (self, msg):
        self.interruption = msg.data
        rospy.loginfo(str(msg.data))
        self.done = True

    def handleFinished (self, msg):
        self.done = True

    def handleControlNormChanged (self, msg):
        self.control_norm_ok = msg.data < 1e-2

    def execute(self, userdata):
        # TODO Check that there the current SOT and the future SOT are compatible ?
        status = self.serviceProxies['sot']['plug_sot'](userdata.transitionId)
        if not status.success:
            rospy.logerr(status.msg)
            return _outcomes[1]

        self.done = False
        # self.control_norm_ok = False
        self.targetPub["publish"].publish(Empty())
        # Wait for errors or publish done
        rate = rospy.Rate (1000)
        while not self.done:
            if self.error is not None:
                # TODO handle error
                rospy.logerr(str(self.error))
                self.error = None
                return _outcomes[1]
            rate.sleep()
        if self.interruption is not None:
            rospy.logerr(str(self.interruption))
            self.interruption = None
            return _outcomes[2]
        print "Wait for event on /sot_hpp/control_norm_changed"
        while not self.control_norm_ok:
            rate.sleep()
        return _outcomes[0]

class WaitForInput(smach.State):
    serviceProxiesDict = {
            'sot': {
                'request_hpp_topics': [ std_srvs.srv.Trigger, ],
                'plug_sot': [ PlugSot, ]
                },
            'hpp': {
                'target': {
                    "reset_topics": [ std_srvs.srv.Empty, ],
                    }
                }
            }

    def __init__(self):
        super(WaitForInput, self).__init__(
                outcomes = [ "succeeded", "aborted" ],
                input_keys = [ ],
                output_keys = [ "pathId", "times", "currentSection" ],
                )

        self.services = ros_tools.createServices (self, "", self.serviceProxiesDict, serve = False)
        self.hppclient = HppClient (False)

    def execute (self, userdata):
        status = self.services['sot']['plug_sot'](-1)
        res = rospy.wait_for_message ("/sm_sot_hpp/start_path", UInt32)
        pid = res.data
        rospy.loginfo("Requested to start path " + str(pid))
        userdata.pathId = pid
        try:
            hpp = self.hppclient._hpp()
            qs, ts = hpp.problem.getWaypoints(pid)
            # Add a first section to force going to init pose.
            userdata.times = ts[0:1] + ts
            userdata.currentSection = -1
            # TODO this should not be necessary
            self.services['hpp']['target']['reset_topics']()
            self.services['sot']['request_hpp_topics']()
            # TODO check that qs[0] and the current robot configuration are
            # close
        except Exception, e:
            rospy.logerr("Failed " + str(e))
            return "aborted"
        return "succeeded"

def makeStateMachine():
    sm = smach.StateMachine (outcomes = _outcomes)

    with sm:
        smach.StateMachine.add ('WaitForInput', WaitForInput(),
                transitions = {
                    "succeeded": 'Init',
                    "aborted": "WaitForInput" },
                remapping = {
                    "pathId": "pathId",
                    "times": "times",
                    "currentSection": "currentSection",
                    })
        smach.StateMachine.add ('Init', InitializePath(),
                transitions = {
                    "succeeded": "WaitForInput",
                    "aborted": "aborted",
                    "preempted": 'Play'},
                remapping = {
                    "pathId": "pathId",
                    "times": "times",
                    "transitionId": "transitionId",
                    "currentSection": "currentSection",
                    })
        smach.StateMachine.add ('Play', PlayPath(),
                transitions = {
                    "succeeded": 'Init',
                    "aborted": "WaitForInput",
                    "preempted": "WaitForInput"},
                remapping = {
                    "transitionId": "transitionId",
                    })

    sm.set_initial_state(["WaitForInput"])

    sis = smach_ros.IntrospectionServer('sm_sot_hpp', sm, '/SM_SOT_HPP')
    return sm, sis
