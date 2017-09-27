#!/usr/bin/env python
import rospy, hpp.corbaserver
from hpp_ros_interface.msg import ProblemSolved
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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


class PlanningRequestAdapter:
    subscribersDict = {
            "motion_planning": {
                "request" : [JointState, "request" ],
                "param" : {
                    'init_position_mode': [ String, "init_position_mode" ],
                    'set_init_position': [ JointState, "set_init_position" ],
                    },
                },
            }
    publishersDict = {
            "motion_planning": {
                "problem_solved" : [ ProblemSolved, 1],
                },
            }
    modes = [ "current", "user_defined" ]

    def __init__ (self, topicStateFeedback, hpp_url = "corbaloc:iiop:/NameService"):
        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.publishers = self._createTopics ("", self.publishersDict, False)
        self.topicStateFeedback = topicStateFeedback
        self.hpp_url = hpp_url
        self.hpp = hpp.corbaserver.Client(url=hpp_url)
        self.q_init = self.hpp.robot.getCurrentConfig()
        self.init_mode = "user_defined"
        self.get_current_state = None
        self.mutexSolve = Lock()

    def _hpp (self, reconnect = True):
        try:
            self.hpp.robot.getRobotName()
        except (CORBA.TRANSIENT, CORBA.COMM_FAILURE) as e:
            if reconnect:
                rospy.loginfo ("Connection with HPP lost. Trying to reconnect.")
                self.hpp = hpp.corbaserver.Client(url=self.hpp_url)
                return self._hpp(False)
            else: raise e
        return self.hpp

    def _createTopics (self, namespace, topics, subscribe):
        rets = dict ()
        if isinstance(topics, dict):
            for k, v in topics.items():
                rets.update(self._createTopics(namespace + "/" + k, v, subscribe))
        else:
            if subscribe:
                try:
                    callback = getattr(self, topics[1])
                except AttributeError:
                    raise NotImplementedError("Class `{}` does not implement `{}`".format(self.__class__.__name__, topics[1]))
                rets[namespace] = rospy.Subscriber(namespace, topics[0], callback)
            else:
                rets[namespace] = rospy.Publisher(namespace, topics[0], queue_size = topics[1])
        return rets

    def _JointStateToConfig(self, msg):
        hpp = self._hpp()
        hpp.robot.setCurrentConfig(self.q_init)
        for jn, q in zip(msg.name, msg.position):
            size = hpp.robot.getJointConfigSize(jn)
            if size == 2:
                hpp.robot.setJointConfig(jn, [cos(q), sin(q)])
            else:
                hpp.robot.setJointConfig(jn, [q])
        return hpp.robot.getCurrentConfig()

    def request (self, msg):
        self.mutexSolve.acquire()
        try:
            if self.init_mode == "current":
                self.set_init_position(self.last_state)
            hpp = self._hpp()
            # print self.q_init
            hpp.problem.setInitialConfig(self.q_init)
            q_goal = self._JointStateToConfig(msg)
            # print q_goal
            hpp.problem.resetGoalConfigs()
            hpp.problem.addGoalConfig(q_goal)
            t = hpp.problem.solve()
            pid = hpp.problem.numberPaths() - 1
            time = t[0] * 3600 + t[1] * 60 + t[2] + t[3] * 1e-3
            # print "Solved in", t, ", path id", pid
            rospy.loginfo("Path ({}) to reach target found in {} seconds".format(pid, t))
            rospy.sleep(0.1)
            self.publishers["/motion_planning/problem_solved"].publish (ProblemSolved(True, "success", pid))
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
        self.last_state = msg

    def set_init_position(self, msg):
        self.q_init = self._JointStateToConfig(msg)
