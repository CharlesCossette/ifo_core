#!/usr/bin/env python2
from ifo_common.utils import retry_if_false
import rospy
from mavros_msgs.srv import (
    SetMode,
    CommandBool,
    ParamSet,
    ParamSetRequest,
    ParamGet,
    MessageInterval,
)

from mavros_msgs.msg import State, PositionTarget, ExtendedState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
from threading import Thread
from ifo_common.ifo_node import IfoNode
from tf.transformations import euler_from_quaternion

# Mavlink message IDs
HIGHRES_IMU = 105
DISTANCE_SENSOR = 132
ATTITUDE = 30
ATTITUDE_QUATERNION = 31


class ControllerNode(IfoNode):
    """
    The controller node provides the main interface to mavros. It provides basic
    procedures for takeoff, landing, altitude holding, and reaching waypoints.

    Currently, it is mainly built under the assumption that mocap data is being
    streamed to the drone regularly.

    This class starts a thread which continuously publishes its own
    `setpoint_msg` property on the /mavros/setpoint_raw/local topic.

    The command-sending has been abstracted away into

        self.set_velocity_command(vx, vy, vz, yaw_rate)
        self.set_position_command(px, py, pz, yaw)
    """

    def __init__(self):
        super(ControllerNode, self).__init__("controller", republish=True)

        self.thread_ready = False
        self.kill_thread = False
        self.ready_topics = {key: False for key in ["pose"]}
        self.report_diagnostics(level=1, message="Initializing.")

        # ----------------------------- Services ------------------------------#
        # Wait for services to become available
        service_timeout = 20
        try:
            rospy.wait_for_service("mavros/param/set", service_timeout)
            rospy.wait_for_service("mavros/param/get", service_timeout)
            rospy.wait_for_service("mavros/set_message_interval", service_timeout)
            self._services_online = True
        except rospy.ROSException:
            rospy.logwarn("IFO CORE: Controller required services not online.")
            self._services_online = False

        # Create service proxies
        self.set_param_srv = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self.get_param_srv = rospy.ServiceProxy("mavros/param/get", ParamGet)
        self.set_rate_srv = rospy.ServiceProxy(
            "mavros/set_message_interval", MessageInterval
        )

        # --------------------------- Subscribers -----------------------------#
        self.pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.cb_mavros_pose
        )
        self.velocity_cmd_sub = rospy.Subscriber(
            "controller/velocity_cmd_in",
            PositionTarget,
            self.cb_velocity_cmd_in,
        )
        self.takeoff_sub = rospy.Subscriber("controller/takeoff", Bool, self.cb_takeoff)
        self.land_sub = rospy.Subscriber("controller/land", Bool, self.cb_land)

        # ---------------------------- Publishers -----------------------------#
        self.setpoint_pub = rospy.Publisher(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        # ---------------------------------------------------------------------#
        # Set sensor/mavlink messaging rates
        self.set_rate(HIGHRES_IMU, 200)
        self.set_rate(DISTANCE_SENSOR, 30)
        self.set_rate(ATTITUDE, 30)  # Reduced to free bandwidth
        self.set_rate(ATTITUDE_QUATERNION, 30)  # Reduced to free bandwidth

        #
        self.setpoint_msg = PositionTarget()
        self.set_position_command(0, 0, 0, 0)
        self.pose = PoseStamped()

        # Send setpoints in seperate thread to better prevent failsafe.
        # If PX4 does not receive a constant stream of setpoints, it will
        # activate a failsafe.
        self.setpoint_thread = Thread(target=self.send_setpoint, args=())
        self.setpoint_thread.daemon = True
        self.setpoint_thread.start()

        rospy.sleep(15)
        self.set_max_xy_speed(0.5)  # TODO: real param.
        self.report_diagnostics(level=0, message="Ready. Idle.")

    def cb_mavros_pose(self, pose_msg):
        self.pose = pose_msg
        self.ready_topics["pose"] = True

    def cb_takeoff(self, msg):
        self.takeoff()

    def cb_land(self, msg):
        self.land()

    def send_setpoint(self):
        """
        This function runs in a seperate thread, sending setpoint commands to
        the FCU at 10Hz. This is required by PX4.

        This is performed by reading the class internal variable `setpoint_msg`,
        and continuously sending its value to the FCU. Killing this thread
        activates a failsafe on the FCU.

        This function is called once.
        """
        rate = rospy.Rate(10)  # Hz
        self.setpoint_msg.header = Header()
        self.setpoint_msg.header.frame_id = "base_footprint"
        self.thread_ready = True  # Thread is ready.
        while not rospy.is_shutdown():
            if not self.killswitch:
                self.setpoint_msg.header.stamp = rospy.Time.now()
                self.setpoint_pub.publish(self.setpoint_msg)
                try:  # prevent garbage in console output when thread is killed
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass
            else:
                rospy.logwarn_once("Halted sending commands to FCU.")

    def preflight_check(self, timeout=30):
        """
        Performs a few pre-flight checks before takeoff.
        """
        loop_freq = 5
        rate = rospy.Rate(loop_freq)
        preflight_passed = False
        for i in range(loop_freq * timeout):
            # Checks:
            # - thread is ready
            # - services online
            # - getting data from all subscribed topics
            if (
                self.thread_ready
                and all(self.ready_topics.values())
                and self._services_online
            ):
                preflight_passed = True
                rospy.loginfo("IFO_CORE: Preflight check passed.")
                break
            rate.sleep()

        if not preflight_passed:
            # TODO: add reason for failure
            rospy.logerr("IFO_CORE: Preflight check failed.")

        return preflight_passed

    def takeoff(self, target_altitude=1, timeout=10):
        """
        Arms the quadcopter, and increases altitude until the target altitude is
        reached. RELIES ON STATE ESTIMATE FEEDBACK.

        Simple proportional control is used.
        """
        rospy.loginfo("Commanding takeoff.")
        preflight_success = self.preflight_check()
        if not preflight_success:
            return False

        mode_success = self.set_mode("OFFBOARD", timeout=3)
        if not mode_success:
            return False

        arm_success = self.set_arm(True)
        if not arm_success:
            return False

        # considered reached when within threshold meters of target altitude
        threshold = 0.2

        loop_freq = 50  # Hz
        rate = rospy.Rate(loop_freq)
        takeoff_successful = False
        for i in range(timeout * loop_freq):
            # Get altitude from state estimator
            altitude = self.pose.pose.position.z

            # Simple proportional control
            k_p = 1.5  # Gain
            u = k_p * (target_altitude - altitude)
            self.set_velocity_command(vz=u)

            if abs(target_altitude - altitude) < threshold:
                takeoff_successful = True
                rospy.loginfo("IFO_CORE: Takeoff successful, altitude reached.")
                self.set_velocity_command(vz=0)
                break

            rate.sleep()
        return takeoff_successful

    def hold_altitude(self, target_altitude=None, duration=1):
        """
        Holds the quadcopter at current altitude unless overridden by arguments.
        Uses a simple proportional feedback law, and will keep updating the
        control command for a user-specified duration.

        USES STATE ESTIMATE FEEDBACK. WARNING: X,Y,YAW will drift as these are
        left uncontrolled/open loop.
        """

        loop_freq = 50  # Hz
        rate = rospy.Rate(loop_freq)
        altitude = self.pose.pose.position.z
        if target_altitude is None:
            target_altitude = altitude

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < duration:
            # Get altitude from state estimator
            altitude = self.pose.pose.position.z

            # Simple proportional control
            k_p = 1.5  # Gain
            u = k_p * (target_altitude - altitude)
            self.set_velocity_command(vz=u)
            rate.sleep()

        self.set_velocity_command(0, 0, 0, 0)

    def hold_position(self, px=None, py=None, pz=None, yaw=None):
        """
        Sets the position command to current position, unless overridden by arguments.

        USES STATE ESTIMATE FEEDBACK.
        """

        loop_freq = 50  # Hz
        rate = rospy.Rate(loop_freq)
        if px is None:
            px = self.pose.pose.position.x
        if py is None:
            py = self.pose.pose.position.y
        if pz is None:
            pz = self.pose.pose.position.z
        if yaw is None:
            q = self.pose.pose.orientation
            (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw = float(y)

        self.set_position_command(px, py, pz, yaw)

    def land(self):
        """
        Lands the quadcopter where it is.
        """
        self.set_velocity_command(vz=-0.6)  # Send downwards velocity command.
        self.set_mode(
            "AUTO.LAND", timeout=5
        )  # Simulanteously activate automatic landing.
        self.report_diagnostics(level=0, message="Normal. Landing.")
        is_landed = self.wait_for_landed_state(timeout=10)  # This is failing often.
        if is_landed:
            self.set_arm(False)  # Landed state required for disarming.
        else:
            rospy.logwarn(
                "Automatic landing failed. Sending downwards velocity command."
            )
            self.report_diagnostics(level=1, message="Landing issue.")
            self.set_velocity_command(vz=-1.5)
            is_landed = self.wait_for_landed_state(timeout=3)
            if is_landed:
                self.set_arm(False)
            else:
                rospy.logwarn("Landing state still not detected. Killing.")
                self.report_diagnostics(level=2, message="Landing failed. Killing")
                self.killswitch = True
                self.kill_motors()

    @retry_if_false(timeout=3, freq=1, log_msg="IFO CORE | set rate")
    def set_rate(self, message_id, rate):
        """
        Sets the FCU flight mode.

        mode: PX4 mode string
        """

        try:
            res = self.set_rate_srv(message_id, rate)  # 0 is custom mode
            if res.success:
                return True
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False

    @retry_if_false(timeout=1, freq=10, log_msg="IFO CORE | set max speed")
    def set_max_xy_speed(self, speed):
        """
        Sets the maximum horizontal travelling speed of the quadcopter.
        """
        req = ParamSetRequest()
        req.param_id = "MPC_XY_VEL_MAX"
        req.value.real = speed
        _ = self.set_param_srv(req)
        res = self.get_param_srv(param_id="MPC_XY_VEL_MAX")
        if abs(res.value.real - speed) < 1e-5:
            return True
        else:
            return False

    def set_velocity_command(self, vx=0, vy=0, vz=0, yaw_rate=0):
        """
        Sets the velocity command to the internal setpoint_msg variable.
        A seperate thread sends the setpoint_msg to the PX4 controller.
        """
        self.setpoint_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_msg.type_mask = (
            PositionTarget.IGNORE_PX
            + PositionTarget.IGNORE_PY
            + PositionTarget.IGNORE_PZ
            + 0 * PositionTarget.IGNORE_VX
            + 0 * PositionTarget.IGNORE_VY
            + 0 * PositionTarget.IGNORE_VZ
            + PositionTarget.IGNORE_AFX
            + PositionTarget.IGNORE_AFY
            + PositionTarget.IGNORE_AFZ
            + PositionTarget.IGNORE_YAW
            + 0 * PositionTarget.IGNORE_YAW_RATE
        )
        self.setpoint_msg.velocity.x = vx
        self.setpoint_msg.velocity.y = vy
        self.setpoint_msg.velocity.z = vz
        self.setpoint_msg.yaw_rate = yaw_rate

    def set_position_command(self, px=0, py=0, pz=0, yaw=None):
        """
        Sets the position command to the internal setpoint_msg variable.
        A seperate thread sends the setpoint_msg to the PX4 controller.

        If yaw is left unspecified, it will be ignored.
        """
        self.setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_msg.type_mask = (
            PositionTarget.IGNORE_VX
            + PositionTarget.IGNORE_VY
            + PositionTarget.IGNORE_VZ
            + PositionTarget.IGNORE_AFX
            + PositionTarget.IGNORE_AFY
            + PositionTarget.IGNORE_AFZ
            + PositionTarget.IGNORE_YAW_RATE
        )

        if yaw is None:
            self.setpoint_msg.type_mask += PositionTarget.IGNORE_YAW
            yaw = 0

        self.setpoint_msg.position.x = px
        self.setpoint_msg.position.y = py
        self.setpoint_msg.position.z = pz
        self.setpoint_msg.yaw = yaw

    def cb_position_cmd_in(self, position_cmd_msg):
        pass

    def cb_velocity_cmd_in(self, velocity_cmd_msg):
        """
        Allows for an external (body frame) velocity command, which will be
        forwarded to mavros.

        Performs takeoff if an upward velocity command is sent.
        """
        if self.extended_state.landed_state == 1:  # 1 for landed, 0 for not landed
            if velocity_cmd_msg.velocity.z > 0.1:
                self.takeoff()

        self.set_velocity_command(
            velocity_cmd_msg.velocity.x,
            velocity_cmd_msg.velocity.y,
            velocity_cmd_msg.velocity.z,
            velocity_cmd_msg.yaw_rate,
        )
