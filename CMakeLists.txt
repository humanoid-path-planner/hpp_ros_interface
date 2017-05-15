# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# INCLUDE(cmake/base.cmake)
# INCLUDE(cmake/python.cmake)

# SET(PROJECT_NAME hpp_ros_interface)
# SET(PROJECT_DESCRIPTION "")
# SET(PROJECT_URL "")

# SETUP_PROJECT()
PROJECT(hpp_ros_interface)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

add_service_files(FILES
  hpp/corbaserver/problem/hpp_corbaserver_problem_configAtParam.srv
  hpp/corbaserver/problem/hpp_corbaserver_problem_setInitialConfig.srv
  hpp/corbaserver/problem/hpp_corbaserver_problem_addGoalConfig.srv
  hpp/corbaserver/problem/hpp_corbaserver_problem_resetGoalConfigs.srv
  hpp/corbaserver/problem/hpp_corbaserver_problem_solve.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS message_runtime)

INSTALL(PROGRAMS src/hpp_ros.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# SETUP_PROJECT_FINALIZE()
