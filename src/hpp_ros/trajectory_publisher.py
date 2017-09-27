#!/usr/bin/env python
import rospy, hpp.corbaserver
from hpp_ros_interface.msg import ProblemSolved
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

def _fillVector(input, segments):
    """ Returns a vector that contains the segments extracted from input """
    output = []
    for s in segments:
        output.extend (input[s[0]:s[0]+s[1]])
    return output

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
