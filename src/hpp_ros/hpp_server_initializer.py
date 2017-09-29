#!/usr/bin/env python
import rospy,
from hpp.gepetto import ViewerFactory
from hpp.corbaserver import Robot, ProblemSolver

from hpp_ros_interface.msg import ProblemSolved
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Empty
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

python_to_corba = {
        float: CORBA.TC_double,
        int: CORBA.TC_long,
        bool: CORBA.TC_boolean,
        str: CORBA.TC_string
        }

def setParameters (hpp, name, params):
    if isinstance(params, dict):
        for k, v in params.items():
            setParameters (hpp, name + "/" + k, v)
    elif python_to_corba.has_key(type(params)):
        hpp.problem.setParameters(name, CORBA.Any(python_to_corba[type(params)], params))
    else:
        rospy.logwarn("Could not set parameter " + name + " with value " +
                str(params) + " of unknown type " + str(type(params)))

class HppServerInitializer:
    subscribersDict = {
            "hpp": {
                "reset" : [Bool, "reset" ],
                "create_viewer" : [Empty, "createViewer" ],
                "configure" : [Empty, "configure" ],
                "load_environment" : [String, "loadEnvironment" ],
                },
            }

    def __init__ (self, hpp_url = "corbaloc:iiop:/NameService"):
        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.publishers = self._createTopics ("", self.publishersDict, False)
        self.topicStateFeedback = topicStateFeedback
        self.hpp_url = None
        self.setHppUrl()
        self.robot = None
        self.ps = None

    def setHppUrl (self):
        hpphost = rospy.get_param ("/hpp/host", "localhost")
        hppport = rospy.get_param ("/hpp/port", 2809)
        url = "corbaloc:iiop:{}:{}/NameService".format(hpphost, hppport)
        if url != self.hpp_url:
            self.hpp_url = url
            self.hpp = hpp.corbaserver.Client(url=url)

    def reset (self, msg):
        self.initialize()
        self.loadRobot()
        self.loadEnvironment(param="/environment/description", paramName="/environment/name")
        self.configure()
        try:
            self.createViewer()
        except:
            rospy.loginfo("Could not reach the Gepetto-viewer")

    def initialize (self):
        self.setHppUrl()
        self.robot = None
        self.ps = None
        self.vf = None
        hpp = self._hpp()
        # Reset the problem if necessary
        if self.robot is not None:
            hpp.problem.resetProblem()

    def loadRobot(self):
        hpp = self._hpp()

        # Load the robot
        robotName = rospy.get_param ("robot_name", "robot")
        rootJointType = rospy.get_param ("robot_root_joint_type", "anchor")
        urdfString = rospy.get_param ("robot_description")
        srdfString = rospy.get_param ("robot_semantic_description", "")

        hpp.robot.loadRobotModelFromString (robotName, rootJointType, urdfString, srdfString)
        self.robot = Robot(robotName, rootJointType, client = hpp, load=False)
        self.ps = ProblemSolver (self.robot)
        self.vf = ViewerFactory (self.ps)
        vf.loadRobot(urdfString)

    def loadEnvironment(self, param = None, xmlString = None, paramName = None, name = None):
        if name is None:
            name = rospy.get_param(paramName) if paramName is not None else "obstacle"
        if param is not None:
            self.vf.loadObstacleModel("", rospy.get_param(param), name)
        elif xmlString
            self.vf.loadObstacleModel("", xmlString, name)

    def configure (self):
        hpp = self._hpp()

        # Setup the planner
        setParameters (hpp, "", rospy.get_param("/hpp/parameters", dict()))

        if rospy.has_param("/hpp/path_validation/method"):
            ps.selectPathValidation(
                    rospy.get_param("/hpp/path_validation/method"),
                    rospy.get_param("/hpp/path_validation/tolerance", 0.05))
        ps.clearPathOptimizers()
        if rospy.has_param("/hpp/path_optimizers"):
            pathOpts = rospy.get_param("/hpp/path_optimizers", ["RandomShortcut"])
            if isinstance(pathOpts, list):
                for n in pathOpts:
                    ps.addPathOptimizer (n)
            else:
                rospy.logerr("Parameter /hpp/path_optimizers shoud be a list of strings.")

    def createViewer (self, *args, **kwargs):
        host = rospy.get_param("/gepetto_viewer/host", "localhost")
        return self.vf.createViewer(host = host, *args, **kwargs)

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
