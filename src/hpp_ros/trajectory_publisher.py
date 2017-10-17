#!/usr/bin/env python
import rospy, hpp.corbaserver
import numpy as np
from hpp_ros_interface.client import HppClient
from hpp_ros_interface.msg import *
from hpp_ros_interface.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import Queue
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import Vector3, Quaternion, Transform
from std_msgs.msg import UInt32, Empty

def _fillVector(input, segments):
    """ Returns a vector that contains the segments extracted from input """
    output = []
    for s in segments:
        output.extend (input[s[0]:s[0]+s[1]])
    return output

def listToVector3(l):
    return Vector3 (l[0], l[1], l[2])
def listToQuaternion(l):
    return Quaternion (l[0], l[1], l[2], l[3])
def listToTransform(l):
    return Transform(listToVector3(l[0:3]), listToQuaternion(l[3:7]))

def init_node ():
    rospy.init_node('joint_path_command_publisher')


class JointPathCommandPublisher:
    def __init__ (self, topic = 'joint_path_command', hasVelocity = False, client = hpp.corbaserver.Client()):
        self.hpp = client
        self.pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
        self.solved = None
        self.msg = JointTrajectory()
        self.msg.header.frame_id = ""
        self.hasVelocity = hasVelocity
        self.setJointNames(self.hpp.robot.getJointNames())

    def setJointNames (self, jointNames):
        self.msg.joint_names = jointNames
        self.configSegments = list()
        self.velocitySegments = list()
        rkCfg = rkVel = 0
        for jn in jointNames:
            szCfg = self.hpp.robot.getJointConfigSize (jn)
            szVel = self.hpp.robot.getJointNumberDof (jn)
            self.configSegments   += [[rkCfg, szCfg ]]
            self.velocitySegments += [[rkVel, szVel ]]
            rkCfg += szCfg
            rkVel += szVel

    def _makeTrajectoryPoint(self, pathId, t, pointTime, pointId):
        q = self.hpp.problem.configAtParam(pathId, t)
        self.msg.points[pointId].positions = _fillVector (q, self.configSegments)
        if self.hasVelocity:
            v = self.hpp.problem.velocityAtParam(pathId, t)
            self.msg.points[pointId].velocities = _fillVector (v, self.velocitySegments)
        self.msg.points[pointId].time_from_start = rospy.Duration(pointTime)

    def publish(self, pathId, dt = 0.05, scale = 1):
        """
        dt: time between samples of the path in HPP
        scale: ratio (time between ROS points) / dt
        """
        self.msg.points = []
        pathLength = self.hpp.problem.pathLength(pathId)
        t = 0.
        last = False
        while True:
            self.msg.points.append(JointTrajectoryPoint())
            self._makeTrajectoryPoint (pathId, t, scale * t, len(self.msg.points)-1)
            if last: break
            t += dt
            if t > pathLength:
                t = pathLength
                last = True
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)
        self.msg.header.seq += 1

    def solve(self, goal, wait = True):
        if wait:
            self.solved = None
        elif self.solved is None:
            self.solved = rospy.Subscriber('/motion_planning/problem_solved', ProblemSolved, self._solved)
        solve = rospy.Publisher('/motion_planning/request', JointState, latch=True, queue_size=1)
        js = JointState(name = self.msg.joint_names, position = goal)
        solve.publish(js)
        if wait:
            msg = rospy.wait_for_message ('/motion_planning/problem_solved', ProblemSolved)
            if msg.success:
                self.publish(msg.path_id)
            else:
                print "Failed:", msg.msg

    def _solved(self, msg):
        if msg.success:
            print "Solved", msg.path_id
        else:
            print "Failed:", msg.msg

class HppOutputQueue(HppClient):
    subscribersDict = {
            "hpp": {
                "target": {
                    "read_path": [ UInt32, "read" ],
                    "publish": [ Empty, "publish" ]
                    },
                },
            }
    servicesDict = {
            "hpp": {
                "target": {
                    "set_joint_names": [ SetJointNames, "setJointNames", ],
                    "add_center_of_mass": [ SetString, "addCenterOfMass", ],
                    "add_operational_frame": [ SetString, "addOperationalFrame", ],
                    }
                }
            }

    class Topic (object):
        def __init__ (self, reader, topicPub, MsgType, data = None):
            self.reader = reader
            self.pub = rospy.Publisher("/hpp/target/" + topicPub, MsgType, latch=True, queue_size=1)
            self.MsgType = MsgType
            self.data = data

        def read (self, hpp, pathId, time):
            # return self.MsgType (self.reader(hpp, pathId, time, self.data))
            return self.reader(hpp, pathId, time, self.data)

        def publish (self, msg):
            self.pub.publish(msg)
    class SentToViewer (object):
        def __init__ (self, parent):
            self.parent = parent

        def read (self, hpp, pathId, time, uv):
            if uv:
                hpp.robot.setCurrentConfig ( hpp.problem.configAtParam(pathId, time) )
                pos = list()
                for j, prefix, o in self.parent.viewer.robotBodies:
                    pos.append ( (prefix + o, hpp.robot.getLinkPosition (o) ) )
                return tuple(pos)
            else:
                return ()

        def publish (self, msg):
            if not isinstance(msg, (tuple, list)) or len(msg) == 0: return
            for name, pos in msg:
                self.parent.viewer.client.gui.applyConfiguration(name, pos)
            self.parent.viewer.client.gui.refresh()

    def __init__ (self):
        super(HppOutputQueue, self).__init__ (withViewer = True)

        self.frequency = 100 # Hz
        self.viewerFreq = 25 # Hz
        self.queue = Queue.Queue (100)

        self.topics = [
                self.SentToViewer (self),
                self.Topic (self._readConfigAtParam, "joint_state", Vector),
                ]
        self.setJointNames (SetJointNamesRequest(self._hpp().robot.getJointNames()))

        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.services = self._createServices ("", self.servicesDict, True)

    def addCenterOfMass (self, req):
        # TODO check that com exists
        comName = req.value
        n = "com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMass, n, Vector3, data = comName),
                )
        return SetStringResponse(True)

    def addOperationalFrame (self, req):
        # TODO check that frame exists
        n = "op_frame/" + req.value
        self.topics.append (self.Topic (self._readJointPosition, n, Transform, data = req.value))
        return SetStringResponse(True)

    def setJointNames (self, req):
        try:
            hpp = self._hpp()
            jns = hpp.robot.getJointNames() + [None]
            # list of segments in [config, velocity]
            joint_selection = [ [], [] ]
            # rank in [config, velocity]
            rks = [0, 0]
            segments = None
            for jn in jns:
                szs = [hpp.robot.getJointConfigSize(jn), hpp.robot.getJointNumberDof(jn)] if jn is not None else [0,0]
                if jn in req.names: # print "[in ]", jn
                    if segments is None:
                        segments = [ [rks[0], rks[0] + szs[0]], [rks[1], rks[1] + szs[1]] ]
                    else:
                        for i in range(2): segments[i][1] += szs[i]
                else: # print "[out]", jn
                    if segments is not None: # insert previous segments
                        joint_selection[0].append(segments[0])
                        joint_selection[1].append(segments[1])
                        segments = None
                for i in range(2): rks[i] += szs[i]
            self.jointNames = req.names
            self.joint_selection = joint_selection
            # print self.joint_selection
        except:
            return SetJointNamesResponse(False)
        return SetJointNamesResponse(True)

    def _readConfigAtParam (self, client, pathId, time, data):
        qin = client.problem.configAtParam (pathId, time)
        client.robot.setCurrentConfig(qin)
        qout = list()
        # vout = list()
        for segment in self.joint_selection[0]:
            qout.extend(qin[segment[0]:segment[1]])
        # for segment in self.joint_selection[1]:
            # vout.extend(vin[segment[0]:segment[1]])
        return Vector(qout)

    def _readCenterOfMass (self, client, pathId, time, data):
        if data == "":
            v = client.robot.getComPosition()
        else:
            v = client.robot.getPartialCom(data)
        return listToVector3(v)

    def _readJointPosition (self, client, pathId, time, data):
        t = client.robot.getJointPosition(data)
        return listToTransform(t)

    def readAt (self, pathId, time, uv = False):
        hpp = self._hpp()
        msgs = [ self.topics[0].read (hpp, pathId, time, uv), ]
        for topic in self.topics[1:]:
            msgs.append (topic.read(hpp, pathId, time))
        self.queue.put (msgs, True)

    def publishNext (self):
        msgs = self.queue.get(True)
        for topic, msg in zip(self.topics, msgs):
            topic.publish (msg)
        self.queue.task_done()

    def read (self, msg):
        pathId = msg.data
        from math import ceil, floor
        hpp = self._hpp()
        L = hpp.problem.pathLength(pathId)
        N = int(ceil(L * self.frequency))
        rospy.loginfo("Start reading path {} into {} points".format(pathId, N+1))
        times = np.array(range(N+1), dtype=float) / self.frequency
        times[-1] = L
        Nv = int(ceil(float(self.frequency) / float(self.viewerFreq)))
        updateViewer = [ False ] * (N+1)
        if hasattr(self, "viewer"):
            for i in range(0,len(updateViewer), Nv): updateViewer[i] = True
            updateViewer[-1] = True
        for t, uv in zip(times, updateViewer):
            self.readAt(pathId, t, uv)
        rospy.loginfo("Finish reading path {}".format(pathId))

    def publish(self, empty):
        rospy.loginfo("Start publishing queue")
        rate = rospy.Rate (self.frequency)
        while not self.queue.empty():
            self.publishNext()
            rate.sleep()
        rospy.loginfo("Finish publishing queue")
