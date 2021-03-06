import rospy

def wait_for_service (srv, time = 0.2):
    try:
        rospy.wait_for_service(srv, time)
    except rospy.ROSException:
        rospy.logwarn("Waiting for service: {0}".format(srv))
        rospy.wait_for_service(srv)
        rospy.logwarn("Service {0} found.".format(srv))

def createTopics (object, namespace, topics, subscribe):
    """
    \param subscribe boolean whether this node should subscribe to the topics.
                             If False, this node publishes to the topics.
    """
    if isinstance(topics, dict):
        rets = dict ()
        for k, v in topics.items():
            rets[k] = createTopics(object, namespace + "/" + k, v, subscribe)
        return rets
    else:
        if subscribe:
            try:
                callback = getattr(object, topics[1])
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(object.__class__.__name__, topics[1]))
            return rospy.Subscriber(namespace, topics[0], callback)
        else:
            return rospy.Publisher(namespace, topics[0], queue_size = topics[1])

def createServices (object, namespace, services, serve):
    """
    \param serve boolean whether this node should serve or use the topics.
    """
    if isinstance(services, dict):
        rets = dict ()
        for k, v in services.items():
            rets[k] = createServices(object, namespace + "/" + k, v, serve)
            # rets.update(createServices(topic_name, k, v, serve))
        return rets
    else:
        if serve:
            try:
                callback = getattr(object, services[1])
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(object.__class__.__name__, services[1]))
            return rospy.Service(namespace, services[0], callback)
        else:
            wait_for_service (namespace)
            return rospy.ServiceProxy(namespace, services[0])
