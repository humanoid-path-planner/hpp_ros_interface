import smach, smach_ros, rospy
from std_msgs.msg import UInt32, Empty, String, Float64
from sot_hpp_msgs.msg import *
from sot_hpp_msgs.srv import *
from hpp.ros_interface.client import HppClient
import std_srvs.srv
from hpp.ros_interface import ros_tools

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
                input_keys = [ "pathId", "times", "transitionIds", "endStateIds", "currentSection" ],
                output_keys = [ "transitionId", "endStateId", "currentSection" ],
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

        userdata.transitionId = userdata.transitionIds[userdata.currentSection]
        userdata.endStateId = userdata.endStateIds[userdata.currentSection]

        if rospy.get_param ("/sm_sot_hpp/step_by_step", False):
            rospy.loginfo("Wait for message on /sm_sot_hpp/step.")
            rospy.wait_for_message ("/sm_sot_hpp/step", Empty)

        self.targetPub["read_subpath"].publish (ReadSubPath (userdata.pathId, start, length))
        rospy.loginfo("Start reading subpath. Waiting for one second.")
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
                'run_pre_action': [ PlugSot, ],
                'plug_sot': [ PlugSot, ],
                'run_post_action': [ PlugSot, ],
                'clear_queues': [ std_srvs.srv.Trigger, ],
                'read_queue': [ SetInt, ],
                'stop_reading_queue': [ std_srvs.srv.Empty, ],
                }
            }

    def __init__(self):
        super(PlayPath, self).__init__(
                outcomes = _outcomes,
                input_keys = [ "transitionId", "endStateId", ],
                output_keys = [ ])

        self.targetPub = ros_tools.createTopics (self, "/hpp/target", self.hppTargetPubDict, subscribe=False)
        self.subscribers = ros_tools.createTopics (self, "", self.subscribersDict, subscribe=True)
        self.serviceProxies = ros_tools.createServices (self, "", PlayPath.serviceProxiesDict, serve=False)

        self.done = False
        self.error = None
        self.interruption = None
        self.control_norm_ok = True

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
        rate = rospy.Rate (1000)

        # TODO Check that there the current SOT and the future SOT are compatible ?
        rospy.loginfo("Run pre-action")
        status = self.serviceProxies['sot']['run_pre_action'](userdata.transitionId)
        rospy.sleep(0.5)
        rospy.loginfo("Wait for event on /sot_hpp/control_norm_changed")
        while not self.control_norm_ok:
            rate.sleep()

        rospy.loginfo("Publishing path")
        self.done = False
        self.serviceProxies['sot']['clear_queues']()
        self.targetPub["publish"].publish()
        rospy.sleep(1)

        status = self.serviceProxies['sot']['plug_sot'](userdata.transitionId)
        if not status.success:
            rospy.logerr(status.msg)
            return _outcomes[1]

        # self.control_norm_ok = False
        rospy.loginfo("Read queue")
        self.serviceProxies['sot']['read_queue'](10)
        # Wait for errors or publish done
        while not self.done:
            if self.error is not None:
                # TODO handle error
                rospy.logerr(str(self.error))
                self.error = None
                return _outcomes[1]
            rate.sleep()
        rospy.loginfo("Publishing path done.")
        if self.interruption is not None:
            rospy.logerr(str(self.interruption))
            self.interruption = None
            return _outcomes[2]
        rospy.loginfo("Wait for event on /sot_hpp/control_norm_changed")
        while not self.control_norm_ok:
            rate.sleep()
        self.serviceProxies['sot']['stop_reading_queue']()

        # Run post action if any
        rospy.loginfo("Run post-action")
        status = self.serviceProxies['sot']['run_post_action'](userdata.endStateId)
        rospy.sleep(1)
        rospy.loginfo("Wait for event on /sot_hpp/control_norm_changed")
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
                output_keys = [ "pathId", "times", "transitionIds", "endStateIds", "currentSection" ],
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
            manip = self.hppclient._manip()
            qs, ts = hpp.problem.getWaypoints(pid)
            # Add a first section to force going to init pose.
            ts = ts[0:1] + ts
            tids = [ manip.problem.edgeAtParam(pid, (t0 + t1) / 2) for t0,t1 in zip(ts[:-1], ts[1:]) ]
            print len(qs), len(ts), len(tids)
            # Remove fake transitions
            tts = ts[0:3]
            ttids = tids[0:2]
            tqs = qs[0:2]
            for t, id, q in zip(ts[2:], tids[1:], qs[1:]):
                if ttids[-1] == id:
                    tts[-1] = t
                    tqs[-1] = q
                else:
                    ttids.append (id)
                    tts.append(t)
                    tqs.append(q)

            userdata.times = tts
            userdata.transitionIds = ttids
            userdata.endStateIds = tuple ([manip.graph.getNode(q) for q in tqs])
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
                    "transitionIds": "transitionIds",
                    "endStateIds": "endStateIds",
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
