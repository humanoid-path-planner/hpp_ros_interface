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

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp_ros_interface)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "")
SET(PROJECT_VERSION "")

SET(INSTALL_DOCUMENTATION OFF)
SETUP_PROJECT()
FINDPYTHON(2.7 REQUIRED EXACT)

find_package(catkin REQUIRED COMPONENTS
  dynamic_graph_bridge_msgs
  geometry_msgs
  sensor_msgs
  sot_hpp_msgs
  std_msgs
  trajectory_msgs)

catkin_package()

ADD_OPTIONAL_DEPENDENCY("hpp-corbaserver")
ADD_OPTIONAL_DEPENDENCY("hpp-manipulation-corba")
ADD_OPTIONAL_DEPENDENCY("dynamic_graph_bridge_msgs")

ADD_SUBDIRECTORY(src)

INSTALL(PROGRAMS
  scripts/hpp_node.py
  scripts/smach_node.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
INSTALL(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

IF(HPP_CORBASERVER_FOUND
   AND EXISTS ${HPP_CORBASERVER_PREFIX}/bin/hppcorbaserver)
  INSTALL(PROGRAMS ${HPP_CORBASERVER_PREFIX}/bin/hppcorbaserver
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ENDIF()
IF(HPP_MANIPULATION_CORBA_FOUND
    AND EXISTS ${HPP_MANIPULATION_CORBA_PREFIX}/bin/hpp-manipulation-server)
  INSTALL(PROGRAMS ${HPP_MANIPULATION_CORBA_PREFIX}/bin/hpp-manipulation-server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ENDIF()

SETUP_PROJECT_FINALIZE()
