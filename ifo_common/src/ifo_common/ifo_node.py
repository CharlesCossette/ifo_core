#!/usr/bin/env python2
import rospy
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from local_diagnostics.srv import GetNodeLevel, GetNodeLevelRequest
from threading import Thread
from mavros_msgs.srv import (
    SetMode,
    CommandBool,
    CommandLong,
    CommandLongRequest,
)

from mavros_msgs.msg import ExtendedState, State
from .utils import retry_if_false


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
        self.kill_motors()

    Moreover, a `killswitch` has been implemented. Whenever any node reports an
    ERROR to the local_diagnostics node, the local_diagnostics publishes a True
    value to the `local_diagnostics/killswitch` topic. Through an inherited callback,
    this sets the internal variable `self.killswitch` to true as soon as this occurs.
    As such, the inheriter can check `self.killswitch` at any time to see if
    the killswitch has been activated.

    If the user chooses by setting republish = True in the
    constructor, the latest diagnostic message will be re-published to the
    local_diagnostics node. This can be used as a "heartbeat" to indicate
    the node is alive and running.
    """

    def __init__(self, node_name, republish=False, wait=True, **init_kwargs):
        """
        Constructor
        """
        super(IfoNode, self).__init__()

        rospy.init_node(node_name, **init_kwargs)

        # Wait for services to become available
        service_timeout = 5
        try:
            rospy.wait_for_service("mavros/set_mode", service_timeout)
            rospy.wait_for_service("mavros/cmd/arming", service_timeout)
            rospy.wait_for_service("local_diagnostics/get_node_level", service_timeout)
            rospy.wait_for_service("mavros/cmd/command", service_timeout)
        except rospy.ROSException:
            if wait:
                rospy.logwarn("Some services not online. Waiting until available.")
                rospy.wait_for_service("mavros/set_mode", service_timeout)
                rospy.wait_for_service("mavros/cmd/arming", service_timeout)
                rospy.wait_for_service("local_diagnostics/get_node_level")
                rospy.wait_for_service("mavros/cmd/command", service_timeout)
            else:
                rospy.logwarn("Some services not online.")

        self.get_node_level_srv = rospy.ServiceProxy(
            "local_diagnostics/get_node_level", GetNodeLevel
        )
        self.set_mode_srv = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.set_arming_srv = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.mavros_cmd_srv = rospy.ServiceProxy("mavros/cmd/command", CommandLong)

        # Publishers.
        self._diagnostics_pub = rospy.Publisher(
            "local_diagnostics/in", DiagnosticArray, queue_size=10, latch=True
        )

        # Subscribers.
        self.ks_sub = rospy.Subscriber(
            "local_diagnostics/killswitch", Bool, self._cb_killswitch
        )
        self.ext_state_data_sub = rospy.Subscriber(
            "mavros/extended_state",
            ExtendedState,
            self._cb_mavros_extended_state,
        )

        self.state_sub = rospy.Subscriber("mavros/state", State, self._cb_mavros_state)

        self.killswitch = False
        self.state = State()
        self.extended_state = ExtendedState()

        if republish:
            self.last_diag_msg = None
            self._heartbeat = Thread(target=self._heartbeat_thread, args=())
            self._heartbeat.daemon = True
            self._heartbeat.start()

    def _heartbeat_thread(self):
        """
        Thread used for periodic re-publishing at 1 Hz.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.last_diag_msg is not None:
                self.last_diag_msg.header.stamp = rospy.Time.now()
                self._diagnostics_pub.publish(self.last_diag_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def _cb_mavros_state(self, state_msg):
        self.state = state_msg

    def _cb_mavros_extended_state(self, extended_state_msg):
        self.extended_state = extended_state_msg

    def _cb_killswitch(self, killswitch_msg):
        """
        We set up a subscriber to the "killswitch" topic from local_diagnostics.
        This sends a message to all nodes, telling them that something is wrong
        and the mission should end asap.
        """
        self.killswitch = killswitch_msg

    def kill_motors(self):
        """
        Immediately kills motors on the drone.

        Taken from https://github.com/PX4/PX4-Autopilot/issues/14465
        """
        req = CommandLongRequest()
        req.broadcast = False
        req.command = 400
        req.confirmation = 0
        req.param1 = 0.0
        req.param2 = 21196.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        self.mavros_cmd_srv(req)

    @retry_if_false(timeout=3, freq=2, log_msg="IFO CORE | set mode")
    def set_mode(self, mode):
        """
        Sets the FCU flight mode.

        mode: PX4 mode string
        """

        if self.state.mode == mode:
            return True
        else:
            try:
                res = self.set_mode_srv(0, mode)  # 0 is custom mode
                if not res.mode_sent:
                    rospy.logerr("IFO_CORE: failed to send mode command.")
                    return False
                else:
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)
                return False

    @retry_if_false(timeout=5, freq=1, log_msg="IFO CORE | set arm")
    def set_arm(self, arm):
        """
        Sets the arming mode of the FCU.
        arm: True to arm or False to disarm
        """
        if self.state.armed == arm:
            return True
        else:
            try:
                res = self.set_arming_srv(arm)
                if res.success:
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(e)
                return False

    @retry_if_false(timeout=5, freq=5, log_msg="IFO_CORE: landed state confirmed: ")
    def wait_for_landed_state(self):
        if self.extended_state.landed_state == 1:  # 1 for landed, 0 for not landed
            landed_state_confirmed = True
        else:
            landed_state_confirmed = False

        return landed_state_confirmed

    def emergency_land(self, timeout=5):
        """
        Lands the quadcopter where it is.
        """
        self.report_diagnostics(level=1, message="Emergency landing.")
        success = self.set_mode("AUTO.LAND", timeout=1)  # Activate automatic landing.
        landing_success = False
        self.killswitch = True
        if success:
            is_landed = self.wait_for_landed_state(timeout=timeout)
            if is_landed:
                self.report_diagnostics(level=0, message="Landing confirmed.")
                self.set_arm(False)  # Landed state required for disarming.
                landing_success = True

        if not landing_success:
            self.report_diagnostics(
                level=2, message="Graceful emergency landing failed. Killing motors."
            )
            self.kill_motors()

    def report_diagnostics(
        self, level=0, message="", hardware_id="", values=[], name=None
    ):
        """
        Send a diagnostic status update to the local_diagnostics node.
        At the minimum, you should specify `level` and `message` arguments.

        PARAMETERS:
        -----------
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
        name: string
            node name. leave blank to automatically read name.
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

        if level == 0:
            rospy.loginfo(message)
        elif level == 1:
            rospy.logwarn(message)
        elif level == 2:
            rospy.logerr(message)

    def get_node_level(self, node_name):
        """
        Gets the current node level of any other node currently running.
        """
        # TODO. Handle if not doesnt exist.
        request = GetNodeLevelRequest()
        request.node_name = node_name
        return self.get_node_level_srv(request)

    def wait_for_nodes(self, node_names, polling_frequency=1):
        """
        Waits for a specific node(s) to report a level of 0.
        """
        # TODO: what if the age is very old?
        rospy.loginfo("Waiting for " + str(node_names) + " to be ready.")
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

        rospy.loginfo("Node(s) " + str(node_names) + " are ready.")
