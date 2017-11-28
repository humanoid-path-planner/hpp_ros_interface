#!/usr/bin/env python
import rospy, hpp.corbaserver
import numpy as np
from .client import HppClient
from hpp_ros_interface.msg import *
from hpp_ros_interface.srv import *
import ros_tools
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import Queue
from collections import deque
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import Vector3, Quaternion, Transform
from std_msgs.msg import UInt32, Empty
import std_srvs.srv

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
                    "read_subpath": [ ReadSubPath, "readSub" ],
                    "publish": [ Empty, "publish" ]
                    },
                },
            }
    publishersDist = {
            "read_path_done": [ UInt32, 1 ],
            "publish_done": [ Empty, 1 ]
            }
    servicesDict = {
            "hpp": {
                "target": {
                    "set_joint_names": [ SetJointNames, "setJointNames", ],
                    "reset_topics": [ std_srvs.srv.Empty, "resetTopics", ],
                    "add_center_of_mass": [ SetString, "addCenterOfMass", ],
                    "add_operational_frame": [ SetString, "addOperationalFrame", ],
                    "add_center_of_mass_velocity": [ SetString, "addCenterOfMassVelocity", ],
                    "add_operational_frame_velocity": [ SetString, "addOperationalFrameVelocity", ],
                    }
                }
            }

    class Topic (object):
        def __init__ (self, reader, topicPub, MsgType, data = None):
            self.reader = reader
            self.pub = rospy.Publisher("/hpp/target/" + topicPub, MsgType, latch=True, queue_size=1000)
            self.MsgType = MsgType
            self.data = data

        def init (self, hpp):
            msg = self.read (hpp)
            self.pub.publish(msg)

        def read (self, hpp):
            # return self.MsgType (self.reader(hpp, pathId, time, self.data))
            return self.reader(hpp, self.data)

        def publish (self, msg):
            self.pub.publish(msg)
    class SentToViewer (object):
        def __init__ (self, parent):
            self.parent = parent

        def read (self, hpp, uv):
            if uv:
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

        self.frequency = 200 # Hz
        self.viewerFreq = 25 # Hz
        self.queue_size = 10 * self.frequency
        self.queue = Queue.Queue (self.queue_size)
        self.queueViewer = deque ()

        self.setJointNames (SetJointNamesRequest(self._hpp().robot.getJointNames()))

        self.subscribers = self._createTopics ("", self.subscribersDict, True)
        self.services = self._createServices ("", self.servicesDict, True)
        self.pubs = ros_tools.createTopics(self, "/hpp/target", self.publishersDist, subscribe = False)
        self.reading = False

        self.resetTopics ()

    def resetTopics (self, msg = None):
        self.topicViewer = self.SentToViewer (self)
        self.topics = [
                self.Topic (self._readConfigAtParam  , "position", Vector),
                self.Topic (self._readVelocityAtParam, "velocity", Vector),
                ]
        hpp = self._hpp()
        self.topics[0].init(hpp)
        self.topics[1].init(hpp)
        rospy.loginfo("Reset topics")
        if msg is not None:
            return std_srvs.srv.EmptyResponse()

    def addCenterOfMass (self, req):
        # TODO check that com exists
        comName = req.value
        n = "com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMass, n, Vector3, data = comName),
                )
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def addCenterOfMassVelocity (self, req):
        # TODO check that com exists
        comName = req.value
        n = "velocity/com"
        if comName != "":
            n += "/" + comName
        self.topics.append (
                self.Topic (self._readCenterOfMassVelocity, n, Vector3, data = comName),
                )
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def addOperationalFrame (self, req):
        # TODO check that frame exists
        n = "op_frame/" + req.value
        self.topics.append (self.Topic (self._readJointPosition, n, Transform, data = req.value))
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
        return SetStringResponse(True)

    def addOperationalFrameVelocity (self, req):
        # TODO check that frame exists
        n = "velocity/op_frame/" + req.value
        self.topics.append (self.Topic (self._readJointVelocity, n, Vector, data = req.value))
        self.topics[-1].init(self._hpp())
        rospy.loginfo("Add topic " + n)
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
                if jn in req.names:
                    if segments is None:
                        segments = [ [rks[0], rks[0] + szs[0]], [rks[1], rks[1] + szs[1]] ]
                    else:
                        for i in range(2): segments[i][1] += szs[i]
                else:
                    if segments is not None: # insert previous segments
                        joint_selection[0].append(segments[0])
                        joint_selection[1].append(segments[1])
                        segments = None
                for i in range(2): rks[i] += szs[i]
            self.jointNames = req.names
            self.joint_selection = joint_selection
        except:
            return SetJointNamesResponse(False)
        rospy.loginfo("Joint names set to " + str(self.jointNames))
        return SetJointNamesResponse(True)

    def _readConfigAtParam (self, client, data):
        qin = client.robot.getCurrentConfig()
        qout = list()
        for segment in self.joint_selection[0]:
            qout.extend(qin[segment[0]:segment[1]])
        return Vector(qout)

    def _readVelocityAtParam (self, client, data):
        vin = client.robot.getCurrentVelocity()
        vout = list()
        for segment in self.joint_selection[1]:
            vout.extend(vin[segment[0]:segment[1]])
        return Vector(vout)

    def _readCenterOfMass (self, client, data):
        if data == "":
            v = client.robot.getCenterOfMass()
        else:
            v = client.robot.getPartialCom(data)
        return listToVector3(v)

    def _readCenterOfMassVelocity (self, client, data):
        if data == "":
            v = client.robot.getCenterOfMassVelocity()
        else:
            v = client.robot.getVelocityPartialCom(data)
        return listToVector3(v)

    def _readJointPosition (self, client, data):
        t = client.robot.getJointPosition(data)
        return listToTransform(t)

    def _readJointVelocity (self, client, data):
        t = client.robot.getJointVelocity(data)
        return Vector(t)

    def readAt (self, pathId, time, uv = False):
        hpp = self._hpp()
        hpp.robot.setCurrentConfig( hpp.problem.configAtParam (pathId, time))
        hpp.robot.setCurrentVelocity( hpp.problem.velocityAtParam (pathId, time))
        if uv:
            self.queueViewer.append ((time, self.topicViewer.read (hpp, uv)))
        msgs = []
        for topic in self.topics:
            msgs.append (topic.read(hpp))
        self.queue.put (msgs, True)

    def publishNext (self):
        msgs = self.queue.get(True)
        for topic, msg in zip(self.topics, msgs):
            topic.publish (msg)
        self.queue.task_done()

    def publishViewerAtTime (self, time):
        if hasattr(self, "viewer"):
            while len(self.queueViewer) > 0:
                # There is no message in queueViewer
                t, msg = self.queueViewer[0]
                if t < time - 1. / self.frequency:
                    self.topicViewer.publish (msg)
                    self.queueViewer.popleft()
                else:
                    break

    def _read (self, pathId, start, L):
        from math import ceil, floor
        N = int(ceil(abs(L) * self.frequency))
        rospy.loginfo("Start reading path {} (t in [ {}, {} ]) into {} points".format(pathId, start, start + L, N+1))
        self.reading = True
        self.queue = Queue.Queue (self.queue_size)
        times = (-1 if L < 0 else 1 ) *np.array(range(N+1), dtype=float) / self.frequency
        times[-1] = L
        times += start
        Nv = int(ceil(float(self.frequency) / float(self.viewerFreq)))
        updateViewer = [ False ] * (N+1)
        if hasattr(self, "viewer"):
            for i in range(0,len(updateViewer), Nv): updateViewer[i] = True
            updateViewer[-1] = True
        for t, uv in zip(times, updateViewer):
            self.readAt(pathId, t, uv)
        self.pubs["read_path_done"].publish(UInt32(pathId))
        rospy.loginfo("Finish reading path {}".format(pathId))
        self.reading = False

    def read (self, msg):
        pathId = msg.data
        hpp = self._hpp()
        L = hpp.problem.pathLength(pathId)
        self._read (pathId, 0, L)

    def readSub (self, msg):
        self._read (msg.id, msg.start, msg.length)

    def publish(self, empty):
        rospy.loginfo("Start publishing queue (size is {})".format(self.queue.qsize()))
        i = 0
        rate = rospy.Rate (5*self.frequency)
        while not self.queue.empty() or self.reading:
            while not self.queue.empty():
                self.publishNext()
                i += 1
                rate.sleep()
            rate.sleep()
        if self.reading:
            rospy.logwarn("Stop publishing while still reading. Consider increasing the queue.")
        self.pubs["publish_done"].publish(Empty())
        rospy.loginfo("Finish publishing queue ({})".format(i))
