#!/usr/bin/env python
import rospy
from tf import TransformListener
from hpp.gepetto import ViewerFactory
import hpp.corbaserver, hpp.corbaserver.robot
# from hpp.corbaserver import Robot, ProblemSolver

from .client import HppClient
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

class HppServerInitializer(HppClient):
    subscribersDict = {
            "hpp": {
                "reset" : [Bool, "reset" ],
                "create_viewer" : [Empty, "_createViewer" ],
                "configure" : [Empty, "configure" ],
                "load_environment" : [String, "loadEnvironment" ],
                },
            }

    def __init__ (self):
        super(HppServerInitializer, self).__init__ ()
        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.setHppUrl()
        self.robot = None
        self.ps = None
        self.tfListener = TransformListener()
        self.world_frame = "/world"

    def reset (self, msg):
        self.initialize()
        self.loadRobot()
        if msg.data:
          self.loadEnvironment(param="/environment/description", paramName="/environment/name")
        self.configure()
        try:
            self.createViewer()
        except:
            rospy.loginfo("Could not reach the Gepetto-viewer")

    def initialize (self):
        self.setHppUrl()
        hpp = self._hpp()
        # Reset the problem if necessary
        if self.robot is not None:
            hpp.problem.resetProblem()
        self.robot = None
        self.ps = None
        self.vf = None

    def loadRobot(self):
        hppClient = self._hpp()

        # Load the robot
        robotName = rospy.get_param ("/robot_name", "robot")
        rootJointType = rospy.get_param ("/robot_root_joint_type", "anchor")
        urdfString = rospy.get_param ("/robot_description")
        srdfString = rospy.get_param ("/robot_description_semantic", "")

        hppClient.robot.loadRobotModelFromString (robotName, rootJointType, urdfString, srdfString)
        self.robot = hpp.corbaserver.robot.Robot(robotName, rootJointType, client = hppClient, load=False)
        self.ps = hpp.corbaserver.ProblemSolver (self.robot)
        self.vf = ViewerFactory (self.ps)
        self.vf.loadRobot(urdfString)

        # Set the root joint position in the world.
        base = self.robot.getLinkNames("root_joint")[0]
        p, q = self.tfListener.lookupTransform(self.world_frame, base, rospy.Time(0))
        self.robot.setJointPosition ("root_joint", p + q)

    def loadEnvironment(self, param = None, xmlString = None, paramName = None, name = None):
        if name is None:
            name = rospy.get_param(paramName) if paramName is not None else "obstacle"
        if param is not None:
            self.vf.loadObstacleModel("", rospy.get_param(param), name)
        elif xmlString is not None:
            self.vf.loadObstacleModel("", xmlString, name)

    def configure (self):
        hpp = self._hpp()

        # Setup the planner
        setParameters (hpp, "", rospy.get_param("/hpp/parameters", dict()))

        if rospy.has_param("/hpp/path_validation/method"):
            ps.selectPathValidation(
                    rospy.get_param("/hpp/path_validation/method"),
                    rospy.get_param("/hpp/path_validation/tolerance", 0.05))
        self.ps.clearPathOptimizers()
        if rospy.has_param("/hpp/path_optimizers"):
            pathOpts = rospy.get_param("/hpp/path_optimizers", ["RandomShortcut"])
            if isinstance(pathOpts, list):
                for n in pathOpts:
                    self.ps.addPathOptimizer (n)
            else:
                rospy.logerr("Parameter /hpp/path_optimizers shoud be a list of strings.")

    def _createViewer (self, msg):
        self.createViewer()

    def createViewer (self, *args, **kwargs):
        host = rospy.get_param("/gepetto_viewer/host", "localhost")
        try:
            return self.vf.createViewer(host = host, *args, **kwargs)
        except:
            rospy.loginfo("Could not reach the Gepetto-viewer")
