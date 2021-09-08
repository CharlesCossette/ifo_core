import rospy
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from local_diagnostics.srv import GetNodeLevel, GetNodeLevelRequest
from threading import Thread

# TODO: Add a self.wait_for_topics(topic_list) method.
# TODO: Add custom logging functions that wrap rospy.loginfo, rospy.logwarn, etc

class IfoNode(object):
    """
    The IfoNode is an abstract node pattern that should be inherited by all 
    nodes running in the IFO core stack. The purpose of this class is to 
    standardize, and make available, some general-purpose methods to be used
    frequently throughout the code base.

    The main example of this is to report diagnostics in a standard way.
    Currently, I have coded things up such that each node must periodically 
    publish an array of DiagnosticStatus messages to the topic 

        'local_diagnostics/in'

    The `local_diagnostics` node subcribes to this topic, aggregates all the 
    info from all the other nodes, and provides some services. As such, some key
    inherited methods from this class are:

        self.report_diagnostics(**kwargs)
        self.wait_for_node(node_name)
        self.get_node_level()

    Moreover, a `killswitch` has been implemented. Whenever any node reports an 
    ERROR to the local_diagnostics node, the local_diagnostics publishes a True 
    value to the `local_diagnostics/killswitch` topic. Through an inherited callback,
    this sets the internal variable `self.killswitch` to true as soon as this occurs.
    As such, the inheriter can check `self.killswitch` at any time to see if 
    the killswitch has been activated.
    """
    def __init__(self, diagnostics_thread = False):
        """
        If the user chooses by setting diagnostics_thread = True in the 
        constructor, the latest diagnostic message will be re-published to the
        local_diagnostics node. This can be used as a "heartbeat" to indicate
        the node is alive and running.
        """
        super(IfoNode, self).__init__()

        try:
            # Diagnostics publisher.
            self._diagnostics_pub = rospy.Publisher(
                'local_diagnostics/in', DiagnosticArray, queue_size=10,
                latch=True
            )
        except:
            rospy.logerr('You must initialize the node before calling '\
                         + 'super().__init__()')
            raise RuntimeError('You must initialize the node before calling '\
                         + 'super().__init__()')

        # Wait for services to become available
        service_timeout = 10
        try:
            rospy.wait_for_service('local_diagnostics/get_node_level', service_timeout)
        except rospy.ROSException:
            rospy.logwarn('local_diagnostics services not online.' \
                           + 'waiting until available.')
            rospy.wait_for_service('local_diagnostics/get_node_level')

        self.get_node_level_srv = rospy.ServiceProxy('local_diagnostics/get_node_level', GetNodeLevel)

        # Killswitch subscriber.
        self.ks_sub = rospy.Subscriber('local_diagnostics/killswitch', Bool, self._cb_killswitch)
        self.killswitch = False

        if diagnostics_thread:
            self.last_diag_msg = None
            self._report_thread = Thread(target=self._reporting_thread, args=())
            self._report_thread.daemon = True
            self._report_thread.start()

    def _reporting_thread(self):
        """
        Thread used for periodic re-publishing at 1 Hz.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.last_diag_msg is not None:
                self.last_diag_msg.header.stamp = rospy.Time.now()
                self._diagnostics_pub.publish(self.last_diag_msg)
            rate.sleep()

    def _cb_killswitch(self, killswitch_msg):
        """
        We set up a subscriber to the "killswitch" topic from local_diagnostics.
        This sends a message to all nodes, telling them that something is wrong
        and the mission should end asap.
        """
        self.killswitch = killswitch_msg

    def report_diagnostics(self, name = None, level = 0, message = '', 
                           hardware_id ='', values = []):
        """
        Send a diagnostic status update to the local_diagnostics node.
        At the minimum, you should specify `level` and `message` arguments.

        PARAMETERS:
        -----------
        name: string
            node name. leave blank to automatically read name.
        level: int
            0 = OK
            1 = WARN
            2 = ERROR
            3 = STALE
            4 = invalid. some other error in the diagnostic reporting system itself.
        message: string
            custom message to send and display
        hardware_id: string
            usually unused, can specify ID if needed
        values: dict
            any other key-value information to send.
        """
        if name is None:
            name = rospy.get_name()

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        diag_msg.status.append(DiagnosticStatus())
        diag_msg.status[0].name = name
        diag_msg.status[0].level = level
        diag_msg.status[0].message = message
        diag_msg.status[0].hardware_id = hardware_id
        diag_msg.status[0].values = values
        self.last_diag_msg = diag_msg
        self._diagnostics_pub.publish(diag_msg)    

    def get_node_level(self, node_name):
        """
        Gets the current node level of any other node currently running.
        """
        # TODO. Handle if not doesnt exist.
        request = GetNodeLevelRequest()
        request.node_name = node_name
        return self.get_node_level_srv(request)

    def wait_for_nodes(self, node_names, polling_frequency = 1):
        """
        Waits for a specific node(s) to report a level of 0.
        """
        rospy.loginfo('Waiting for ' + str(node_names) + ' to be ready.')
        rate = rospy.Rate(polling_frequency)
        nodes_ready = False
        if type(node_names) == str: 
            node_names = [node_names]
        
        ns = rospy.get_namespace()
        while not nodes_ready:
            nodes_ready = True
            for node_name in node_names:
                response = self.get_node_level(ns + node_name)
                if response.level != 0 and response.is_valid:
                    nodes_ready = False
            rate.sleep()
        
        rospy.loginfo('Node(s) ' + str(node_names) + ' are ready.')
        

    