#!/usr/bin/env python

import sys
import rospy
from hpp_ros_interface.srv import *

class _WrapOutput:
    def __init__ (self, func):
        self.f = func

    def __call__ (self, *args):
        ret = self.f(*args)
        return ret.__getstate__()

class Problem:
    def __init__(self):
        self._addService ("configAtParam", hpp_corbaserver_problem_configAtParam)

    def _addService(self, name, srv):
        rospy.wait_for_service('hpp/corbaserver/problem/' + name)
        self.__dict__[name] = _WrapOutput(rospy.ServiceProxy('hpp/corbaserver/problem/' + name, srv))

def configAtParam(pId, t):
    rospy.wait_for_service('hpp/corbaserver/problem/configAtParam')
    try:
        func = rospy.ServiceProxy('hpp/corbaserver/problem/configAtParam', hpp_corbaserver_problem_configAtParam)
        resp = func(pId, t)
        return resp.ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    if len(sys.argv) == 3:
        id = int(sys.argv[1])
        t = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    problem = Problem()
    print "Requesting ", id, t
    ret, = problem.configAtParam(id, t)
    print "Return ", ret
