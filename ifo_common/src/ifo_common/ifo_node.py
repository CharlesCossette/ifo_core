import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from local_diagnostics.srv import GetNodeLevel, GetNodeLevelRequest
from threading import Thread
class IfoNode(object):
    def __init__(self, diagnostics_thread = False):
        super(IfoNode, self).__init__()

        try:
            self._diagnostics_pub = rospy.Publisher(
                'local_diagnostics/in', DiagnosticArray, queue_size=10,
                latch=True
            )
        except:
            rospy.logerr('You must initialize the node before calling '\
                         + 'super().__init__()')

        # Wait for services to become available
        service_timeout = 10
        try:
            rospy.wait_for_service('local_diagnostics/get_node_level', service_timeout)
        except rospy.ROSException:
            rospy.logwarn('local_diagnostics services not online.')
        
        self.get_node_level_srv = rospy.ServiceProxy('local_diagnostics/get_node_level', GetNodeLevel)

        if diagnostics_thread:
            self.last_diag_msg = None
            self._report_thread = Thread(target=self._reporting_thread, args=())
            self._report_thread.daemon = True
            self._report_thread.start()

    def _reporting_thread(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.last_diag_msg is not None:
                self.last_diag_msg.header.stamp = rospy.Time.now()
                self._diagnostics_pub.publish(self.last_diag_msg)
            rate.sleep()

    def report_diagnostics(self, name = None, level = 0, message = '', 
                           hardware_id ='', values = []):
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

   

    def cb_killswitch(self, killswitch_msg):
        """
        We set up a subscriber to the "killswitch" topic from local_diagnostics.
        This sends a message to all nodes, telling them that something is wrong
        and the mission should end asap.
        """
        self.killswitch = killswitch_msg

    def get_node_level(self, node_name):
        request = GetNodeLevelRequest()
        request.node_name = node_name
        return self.get_node_level_srv(request)

    def wait_for_nodes(self, node_names, polling_frequency = 1):
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
        

    