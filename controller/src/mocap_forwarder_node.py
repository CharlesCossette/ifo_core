#!/usr/bin/env python2
import rospy
from mavros_msgs.srv import ParamGet, ParamSet
from mavros_msgs.msg import ParamValue
from geometry_msgs.msg import PoseStamped
from ifo_common.ifo_node import IfoNode

"""
The mocap forwarder node subscribes to mocap data available on the 
/ifoXXX/vrpn_client_node/ifoXXX/pose topic, and eventually forwards it to
/ifoXXX/mavros/vision_pose/pose topic so that PX4 can fuse this information,
and use it for state estimation. 

The node also sets various PX4 parameters to start accepting this info, as well
as downsampling the frequency (40Hz from 200Hz). 

Moreover, the node implements some basic checks of the validity of mocap data 
in real time.
"""

class MocapForwarderNode(IfoNode):
    def __init__(self):
        super(MocapForwarderNode, self).__init__("mocap_forwarder", republish=True)

        self.report_diagnostics(level=1, message="Initializing...")

        # Wait for services to become available
        service_timeout = 30
        try:
            rospy.wait_for_service("mavros/param/set", service_timeout)
            rospy.wait_for_service("mavros/param/get", service_timeout)
            self._services_online = True
        except rospy.ROSException:
            self._services_online = False

        who_am_i = rospy.get_namespace()
        if who_am_i == "/":
            rospy.loginfo(
                rospy.get_name() + " not launched in a namespace. Assuming ifo001."
            )
            who_am_i = "/ifo001/"  # TODO. Would be nice to ditch this.

        self.set_param_srv = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self.get_param_srv = rospy.ServiceProxy("mavros/param/get", ParamGet)
        self.orig_params = {}

        # We require that a vrpn_client_node be running locally on each agent.
        self.pose_sub = rospy.Subscriber(
            "vrpn_client_node" + who_am_i + "pose", PoseStamped, self.cb_mocap_pose
        )
        self.pose_pub = rospy.Publisher(
            "mavros/vision_pose/pose", PoseStamped, queue_size=1
        )
        self.pose = None
        self.last_pose = None
        # Give PX4 some time to be properly initialized before starting.
        rospy.sleep(15)

    def cb_mocap_pose(self, pose_msg):
        # validate the message
        msg_valid = True
        if (
            pose_msg.pose.position.x == 0.0
            or pose_msg.pose.position.y == 0.0
            or pose_msg.pose.position.z == 0.0
        ):
            msg_valid = False

        if msg_valid:
            self.last_pose = self.pose
            self.pose = pose_msg
            self.last_rx_stamp = rospy.get_time()

    def set_parameter(self, param_id, value, timeout=10):
        """
        Sets an FCU parameter and verifies that it has been done. Also saves
        the original parameter value for possible resetting later.
        """
        rospy.loginfo(
            "IFO_CORE: Setting FCU parameter " + param_id + " to: " + str(value)
        )

        # Create legit ParamValue message.
        param_value = ParamValue()
        if isinstance(value, int):
            param_value.integer = value
        else:
            param_value.real = value

        # Get and save the original value
        orig_value, get_success = self.get_parameter(param_id)
        if isinstance(value, int):
            orig_value = orig_value.integer
        else:
            orig_value = orig_value.real
        if get_success:
            if param_id not in self.orig_params:
                self.orig_params[param_id] = orig_value
        else:
            rospy.loginfo("IFO_CORE: Setting FCU parameter " + param_id + ": FAILED")
            return False

        # Loop until the setting is done, with timeout
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        set_success = False
        start_time = rospy.get_time()
        count = 0  # failsafe to not enter infinite loop
        while rospy.get_time() - start_time < timeout and count < 100:
            resp = self.set_param_srv(param_id, param_value)
            value_check, _ = self.get_parameter(param_id, timeout=2)
            if isinstance(value, int):
                value_check = value_check.integer
            else:
                value_check = value_check.real
            if value_check == value:
                rospy.loginfo(
                    "IFO_CORE: Setting FCU parameter " + param_id + ": SUCCESS"
                )
                set_success = True
                break
            rate.sleep()
            count = count + 1

        if not set_success:
            rospy.loginfo("IFO_CORE: Setting FCU parameter " + param_id + ": FAILED")
        return set_success

    def get_parameter(self, param_id, timeout=10):
        """
        Reads a PX4 parameter value.
        """
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            resp = self.get_param_srv(param_id)
            if resp.success:
                break
            rate.sleep()
        return resp.value, resp.success

    def wait_for_first_message(self, timeout=None):
        """
        Waits eternally until the first pose message is obtained.
        """
        self.report_diagnostics(level=1, message="Waiting for mocap data.")
        if timeout is None:
            timeout = 999999.0

        start_time = rospy.get_time()
        rate = rospy.Rate(3)
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            if self.pose is not None:
                break

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def start(self):

        max_mocap_delay = 1  # Seconds. If havnt recieved a new mocap measurement
        # for this duration, we will kill mission.

        # Set the right PX4 parameters for when using mocap
        self.set_parameter("EKF2_HGT_MODE", 3)
        self.set_parameter("EKF2_AID_MASK", 0b000011000)
        self.set_parameter("EKF2_EV_DELAY", 0.0)
        self.set_parameter("EKF2_EV_POS_X", 0.0)
        self.set_parameter("EKF2_EV_POS_Y", 0.0)
        self.set_parameter("EKF2_EV_POS_Z", 0.0)
        self.set_parameter("EKF2_MAG_TYPE", 5)
        self.set_parameter("EKF2_MAG_TYPE", 5)
        self.set_parameter("CAL_MAG0_PRIO", 0)
        self.set_parameter("CAL_MAG1_PRIO", 0)
        self.set_parameter("MAV_ODOM_LP", 1)

        self.wait_for_first_message()
        self.report_diagnostics(level=0, message="Normal. Forwarding mocap data.")
        loop_freq = 40
        rate = rospy.Rate(loop_freq)
        lost_mocap = False
        while not rospy.is_shutdown():

            if rospy.get_time() - self.last_rx_stamp > max_mocap_delay:
                self.report_diagnostics(
                    level=2, message="ERROR. Interruption in mocap data!"
                )
                self.emergency_land()
                lost_mocap = True

            # TODO: check change of last message.
            else:
                self.pose_pub.publish(self.pose)
                if lost_mocap:
                    self.report_diagnostics(level=0, message="Recovered. Forwarding mocap data.")
                    lost_mocap = False

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def shutdown(self):
        """
        Gets executed on shutdown. Resets the PX4 parameters that were set by
        this node.
        """
        for key, value in self.orig_params.items():
            self.set_parameter(key, value)


if __name__ == "__main__":
    node = MocapForwarderNode()
    rospy.on_shutdown(node.shutdown)
    node.start()
