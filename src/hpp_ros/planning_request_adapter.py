#!/usr/bin/env python
import rospy, hpp.corbaserver
from tf import TransformListener
from hpp_ros_interface.client import HppClient
from hpp_ros_interface.msg import ProblemSolved, PlanningGoal
from hpp_ros_interface.trajectory_publisher import JointPathCommandPublisher
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Bool
from math import cos, sin
from threading import Lock
from omniORB import CORBA

def _fillVector(input, segments):
    """ Returns a vector that contains the segments extracted from input """
    output = []
    for s in segments:
        output.extend (input[s[0]:s[0]+s[1]])
    return output

def init_node ():
    rospy.init_node('planning_request_adapter')


class PlanningRequestAdapter(HppClient):
    subscribersDict = {
            "motion_planning": {
                "set_goal" : [PlanningGoal, "set_goal" ],
                "request" : [Empty, "request" ],
                "param" : {
                    'init_position_mode': [ String, "init_position_mode" ],
                    'set_init_pose': [ PlanningGoal, "set_init_pose" ],
                    },
                },
            }
    publishersDict = {
            "motion_planning": {
                "problem_solved" : [ ProblemSolved, 1],
                },
            }
    modes = [ "current", "user_defined" ]

    def __init__ (self, topicStateFeedback):
        super(PlanningRequestAdapter, self).__init__ ()
        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.publishers = self._createTopics ("", self.publishersDict, False)
        self.topicStateFeedback = topicStateFeedback
        self.setHppUrl()
        self.q_init = None
        self.init_mode = "user_defined"
        self.get_current_state = None
        self.tfListener = TransformListener()
        self.mutexSolve = Lock()
        self.world_frame = "/world"
        self.robot_base_frame = None

    def _hpp (self, reconnect = True):
        hpp = super(PlanningRequestAdapter, self)._hpp(reconnect)
        self.robot_base_frame = hpp.robot.getLinkNames("root_joint")[0]
        rootJointType = rospy.get_param ("robot_root_joint_type", "anchor")
        if rootJointType == "anchor":
            self.setRootJointConfig = lambda x : None
        elif rootJointType == "freeflyer":
            self.setRootJointConfig = lambda x : hpp.robot.setJointConfig("root_joint", x)
        elif rootJointType == "planar":
            self.setRootJointConfig = lambda x : hpp.robot.setJointConfig("root_joint", x[0:2] + [x[6]**2 - x[5]**2, 2 * x[5] * x[6]] )
        else:
            self.setRootJointConfig = lambda x : (_ for _ in ()).throw(Exception("parameter robot_root_joint_type must be one of (anchor, freeflyer, anchor) and not " + str(rootJointType)))
        return hpp

    def _JointStateToConfig(self, placement, js_msg):
        hpp = self._hpp()
        if self.q_init is not None:
            hpp.robot.setCurrentConfig(self.q_init)
        self.setRootJointConfig(placement)
        for jn, q in zip(js_msg.name, js_msg.position):
            size = hpp.robot.getJointConfigSize(jn)
            if size == 2:
                hpp.robot.setJointConfig(jn, [cos(q), sin(q)])
            else:
                hpp.robot.setJointConfig(jn, [q])
        return hpp.robot.getCurrentConfig()

    def set_goal (self, msg):
        hpp = self._hpp()
        q_goal = self._JointStateToConfig(msg.base_placement, msg.joint_state)
        hpp.problem.resetGoalConfigs()
        hpp.problem.addGoalConfig(q_goal)

    def request (self, msg):
        self.mutexSolve.acquire()
        try:
            if self.init_mode == "current":
                self.set_init_pose (PlanningGoal(self.last_placement, self.last_joint_state))
            hpp = self._hpp()
            hpp.problem.setInitialConfig(self.q_init)
            t = hpp.problem.solve()
            pid = hpp.problem.numberPaths() - 1
            time = t[0] * 3600 + t[1] * 60 + t[2] + t[3] * 1e-3
            # print "Solved in", t, ", path id", pid
            rospy.loginfo("Path ({}) to reach target found in {} seconds".format(pid, t))
            rospy.sleep(0.1)
            self.publishers["/motion_planning/problem_solved"].publish (ProblemSolved(True, "success", pid))
            if rospy.get_param("/hpp/publish_path", True):
                topic = rospy.get_param("/hpp/topic_robot_controller", "joint_path_command")
                rospy.loginfo("Publish path to " + str(topic))
                jpc = JointPathCommandPublisher(topic = topic)
                jpc.publish(pid)
        except Exception as e:
            rospy.loginfo (str(e))
            rospy.sleep(0.1)
            self.publishers["/motion_planning/problem_solved"].publish (ProblemSolved(False, str(e), -1))
        finally:
            self.mutexSolve.release()

    def init_position_mode(self, msg):
        if msg.data in self.modes:
            if msg.data == self.init_mode: return
            self.init_mode = msg.data
            rospy.loginfo("Initial position mode: %s" % msg.data)
            if msg.data == "current":
                self.get_current_state = rospy.Subscriber (self.topicStateFeedback, JointState, self.get_joint_state)
            else:
                self.get_current_state = None

    def get_joint_state (self, msg):
        self.last_joint_state = msg
        try:
            base = self.hpp.robot.getLinkNames("root_joint")[0]
            p, q = self.tfListener.lookupTransform(self.world_frame, base, rospy.Time(0))
            self.last_placement = p + q
        except:
            pass

    def set_init_pose(self, msg):
        self.q_init = self._JointStateToConfig(msg.base_placement, msg.joint_state)
