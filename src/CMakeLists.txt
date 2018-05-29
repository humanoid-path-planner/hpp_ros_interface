SET (PYTHON_FILE
    trajectory_publisher.py
    planning_request_adapter.py
    hpp_server_initializer.py
    client.py
    ros_tools.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("hpp/ros_interface" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    hpp_server_initializer.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("hpp/ros_interface/manipulation" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    play_path.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("hpp/ros_interface/path_execution_fsm" ${F})
ENDFOREACH()
