import rospy
import hpp.corbaserver
import hpp.corbaserver.manipulation

class HppClient(object):
    def __init__ (self):
        self.hpp_url = None
        self.setHppUrl()

    def setHppUrl (self):
        hpphost = rospy.get_param ("/hpp/host", "localhost")
        hppport = rospy.get_param ("/hpp/port", 2809)
        url = "corbaloc:iiop:{}:{}/NameService".format(hpphost, hppport)
        if url != self.hpp_url:
            self.hpp_url = url
            self._connect()

    def _connect(self):
        self.hpp = hpp.corbaserver.Client(url=self.hpp_url)
        try:
            self.manip = hpp.corbaserver.manipulation.Client (url = self.hpp_url)
        except:
            if hasattr(self, manip): delattr(self, manip) 

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
        if not hasattr(self, manip):
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
        rets = dict ()
        if isinstance(topics, dict):
            for k, v in topics.items():
                rets.update(self._createTopics(namespace + "/" + k, v, subscribe))
        else:
            if subscribe:
                try:
                    callback = getattr(self, topics[1])
                except AttributeError:
                    raise NotImplementedError("Class `{}` does not implement `{}`".format(self.__class__.__name__, topics[1]))
                rets[namespace] = rospy.Subscriber(namespace, topics[0], callback)
            else:
                rets[namespace] = rospy.Publisher(namespace, topics[0], queue_size = topics[1])
        return rets

    def _createServices (self, namespace, services, serve):
        """
        \param serve boolean whether this node should serve or use the topics.
        """
        rets = dict ()
        if isinstance(services, dict):
            for k, v in services.items():
                rets.update(self._createServices(namespace + "/" + k, v, serve))
        else:
            if serve:
                try:
                    callback = getattr(self, services[1])
                except AttributeError:
                    raise NotImplementedError("Class `{}` does not implement `{}`".format(self.__class__.__name__, services[1]))
                rets[namespace] = rospy.Service(namespace, services[0], callback)
            else:
                rets[namespace] = rospy.ServiceProxy(namespace, services[0])
        return rets
