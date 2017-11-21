import rospy, CORBA
import hpp.corbaserver
import hpp.corbaserver.robot
import hpp.corbaserver.manipulation
import hpp.corbaserver.manipulation.robot
import hpp.gepetto
import hpp.gepetto.manipulation
import ros_tools

class HppClient(object):
    def __init__ (self, withViewer = False):
        self.withViewer = withViewer
        self.setHppUrl()

    def setHppUrl (self):
        self._connect()

    def _connect(self):
        self.hpp = hpp.corbaserver.Client()
        try:
            self.manip = hpp.corbaserver.manipulation.Client ()
            self.robot = hpp.corbaserver.manipulation.robot.Robot ()
            self.problemSolver = hpp.corbaserver.manipulation.ProblemSolver(self.robot)
        except Exception, e:
            rospy.logwarn("Could not connect to manipulation server: " + str(e))
            if hasattr(self, "manip"): delattr(self, "manip") 
            self.robot = hpp.corbaserver.robot.Robot()
            self.problemSolver = hpp.corbaserver.ProblemSolver(self.robot)
        rospy.loginfo("Connected to hpp")
        if self.withViewer:
            try:
                from gepetto.corbaserver import Client as GuiClient
                viewerClient = GuiClient ()
                if hasattr(self, "manip"):
                    self.viewer = hpp.gepetto.manipulation.Viewer (self.problemSolver, viewerClient = viewerClient, displayName = self.hpp.robot.getRobotName())
                else:
                    self.viewer = hpp.gepetto.Viewer (self.problemSolver, viewerClient = viewerClient)
                rospy.loginfo("Connected to gepetto-viewer")
            except Exception, e:
                if hasattr(self, "viewer"): delattr(self, "viewer") 
                rospy.logwarn("Could not connect to gepetto-viewer: " + str(e))

    def _hpp (self, reconnect = True):
        try:
            self.hpp.problem.getAvailable("type")
        except (CORBA.TRANSIENT, CORBA.COMM_FAILURE) as e:
            if reconnect:
                rospy.loginfo ("Connection with HPP lost. Trying to reconnect.")
                self._connect()
                return self._hpp(False)
            else: raise e
        return self.hpp

    def _manip (self, reconnect = True):
        if not hasattr(self, "manip"):
            raise Exception("No manip client")
        try:
            self.manip.problem.getAvailable("type")
        except (CORBA.TRANSIENT, CORBA.COMM_FAILURE) as e:
            if reconnect:
                rospy.loginfo ("Connection with HPP lost. Trying to reconnect.")
                self._connect()
                return self._manip(False)
            else: raise e
        return self.manip

    def _createTopics (self, namespace, topics, subscribe):
        """
        \param subscribe boolean whether this node should subscribe to the topics.
                                 If False, this node publishes to the topics.
        """
        return ros_tools.createTopics(self, namespace, topics, subscribe)

    def _createServices (self, namespace, services, serve):
        """
        \param serve boolean whether this node should serve or use the topics.
        """
        return ros_tools.createServices (self, namespace, services, serve)

    def displayConfig(self, q):
        if self.withViewer and hasattr(self, "viewer"):
            self._hpp()
            self.viewer (q)
