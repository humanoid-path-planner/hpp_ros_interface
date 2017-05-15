#!/usr/bin/env python

from hpp.corbaserver import Client

import rospy
from hpp_ros_interface.srv import *

client = Client()

class corbaserver_problem:
    @staticmethod
    def configAtParam(req):
        return hpp_corbaserver_problem_configAtParamResponse(client.problem.configAtParam(req.inPathId, req.atDistance))
    @staticmethod
    def setInitialConfig(req):
        return hpp_corbaserver_problem_setInitialConfigResponse(client.problem.setInitialConfig(req.dofArray))
    @staticmethod
    def addGoalConfig(req):
        return hpp_corbaserver_problem_addGoalConfigResponse(client.problem.setInitialConfig(req.dofArray))
    @staticmethod
    def resetGoalConfigs(req):
        return hpp_corbaserver_problem_resetGoalConfigsResponse(client.problem.resetGoalConfigs())
    @staticmethod
    def solve(req):
        return hpp_corbaserver_problem_solveResponse(client.problem.solve())

def server():
    rospy.init_node("hpp")
    rospy.Service("/hpp/corbaserver/problem/configAtParam",     hpp_corbaserver_problem_configAtParam, corbaserver_problem.configAtParam)
    rospy.Service("/hpp/corbaserver/problem/setInitialConfig",  hpp_corbaserver_problem_setInitialConfig, corbaserver_problem.setInitialConfig)
    rospy.Service("/hpp/corbaserver/problem/addGoalConfig",     hpp_corbaserver_problem_addGoalConfig, corbaserver_problem.addGoalConfig)
    rospy.Service("/hpp/corbaserver/problem/resetGoalConfigs",  hpp_corbaserver_problem_resetGoalConfigs, corbaserver_problem.resetGoalConfigs)
    rospy.Service("/hpp/corbaserver/problem/solve",  hpp_corbaserver_problem_solve, corbaserver_problem.solve)
    rospy.spin()

if __name__ == '__main__':
  try:
    server()
  except rospy.ROSInterruptException:
    pass
