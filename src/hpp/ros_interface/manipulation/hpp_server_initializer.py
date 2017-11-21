#!/usr/bin/env python
import rospy
from tf import TransformListener
from hpp.gepetto.manipulation import ViewerFactory
import hpp.corbaserver.manipulation, hpp.corbaserver.manipulation.robot
# from hpp.corbaserver import Robot, ProblemSolver

from hpp.ros_interface.hpp_server_initializer import HppServerInitializer as _Parent
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

def checkType (varname, var, types):
    if not isinstance(var, types): raise TypeError(msg + "must be of type " + str(types) + " instead of " + str(type(var)))

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

class HppServerInitializer(_Parent):
    def __init__ (self):
        super(HppServerInitializer, self).__init__()

    def reset (self, msg):
        self.initialize()
        self.loadRobot()
        if msg.data:
          self.loadEnvironment(param="/environment/description", paramName="/environment/name")
        self.configure()
        self.createGraph("graph")
        self.createViewer()

    def initialize (self):
        self.setHppUrl()
        hpp = self._hpp()
        # Reset the problem if necessary
        if self.robot is not None:
            hpp.problem.manipulation.resetProblem()
        self.robot = None
        self.ps = None
        self.vf = None

    def loadRobot(self):
        hppClient = self._hpp()

        robotName = rospy.get_param ("/robot_name", "robot")
        self.manip.robot.create(robotName)

        # Load the robot
        robotNames = rospy.get_param ("/robot_names")

        self.robot = hpp.corbaserver.manipulation.robot.Robot(robotName, robotNames[0], rootJointType=None, load=False)
        self.ps = hpp.corbaserver.manipulation.ProblemSolver (self.robot)
        self.vf = ViewerFactory (self.ps)

        robots = rospy.get_param ("/robots")
        for rn in robotNames:
            r = robots[rn]
            type = r["type"] if r.has_key("type") else "robot"
            if type not in ("robot", "humanoid", "object"):
                rospy.logwarn("Param /robots/" + rn + "/type must be one of robot, humanoid or object. Assuming robot")
                type = "robot"
            if type == "humanoid":
                loadFromString = self.manip.robot.insertHumanoidModelFromString
                loadFromPack   = self.manip.robot.insertHumanoidModel
            elif type == "object":
                rospy.logwarn("Currently, there is no difference between type robot and type object.)")
                loadFromString = self.manip.robot.insertRobotModelFromString
                loadFromPack   = self.manip.robot.insertRobotModel
            else:
                loadFromString = self.manip.robot.insertRobotModelFromString
                loadFromPack   = self.manip.robot.insertRobotModel
            try:
                loadFromString (rn, r["root_joint_type"], r["description"], r["description_semantic"])
                self.vf.loadUrdfInGUI (urdfString, rn)
            except KeyError as e1:
                try:
                    loadFromPack (rn, r["root_joint_type"], r["package"], r["urdfName"], r["urdfSuffix"], r["srdfSuffix"])
                    self.vf.loadUrdfInGUI ("package://" + r["package"] + "/urdf/" + r["urdfName"] + r["urdfSuffix"] + ".urdf", rn)
                except KeyError as e2:
                    rospy.logerr ("Could not load robot " + rn + ":\n" + str(e1) + "\n" + str(e2))

            # Set the root joint position in the world.
            # base = self.robot.getLinkNames("root_joint")[0]
            # p, q = self.tfListener.lookupTransform(self.world_frame, base, rospy.Time(0))
            # self.robot.setJointPosition ("root_joint", p + q)

            self.robot.rebuildRanks()

    def loadEnvironment(self, param = None, xmlString = None, paramName = None, name = None):
        if name is None:
            name = rospy.get_param(paramName) if paramName is not None else "obstacle"
        if param is not None:
            xmlString = rospy.get_param(param)
        if xmlString is not None:
            self.manip.robot.loadEnvironmentModelFromString (xmlString, "", name)
            self.vf.loadUrdfObjectsInGUI(xmlString, name)

    def _createGraph(self, msg):
        self.createGraph(msg.data)

    def createGraph(self, paramName):
        graphInfo = rospy.get_param(paramName)
        grippers = rospy.get_param(paramName + "/grippers", self.ps.getAvailable("gripper"))
        objects = list()
        handlePerObjects = list()
        contactPerObjects = list()
        for objName, objDesc in graphInfo["objects"].items():
            objects.append(objName)
            handlePerObjects.append(objDesc["handles"])
            contactPerObjects.append(objDesc["contacts"])
        envNames = rospy.get_param(paramName + "/environment_contacts", self.ps.getAvailable("envcontact"))
        rules = list()
        for r in rospy.get_param(paramName + "/rules", list()):
            checkType ("In rules, param grippers", r["grippers"], (list, tuple))
            checkType ("In rules, param handles", r["handles"], (list, tuple))
            checkType ("In rules, param accept", r["accept"], (bool))
            rules.append(hpp.corbaserver.manipulation.Rule (r["grippers"], r["handles"], r["accept"]))
        self.graph = hpp.corbaserver.manipulation.ConstraintGraph.buildGenericGraph(
                self.robot,
                graphInfo["name"],
                grippers,
                objects,
                handlePerObjects,
                contactPerObjects,
                envNames,
                rules)

    def configure (self):
        super(HppServerInitializer, self).configure()
