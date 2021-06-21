#!/usr/bin/env python2

import rospy
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, ExtendedState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from threading import Thread
from time import sleep

class ControllerNode(object):
    """
    This class starts a thread which continuously publishes its own 
    `setpoint_msg` property on the /mavros/setpoint_raw/local topic.

    As is, the `setpoint_msg` has the following fields:

        self.setpoint_msg.position.x
        self.setpoint_msg.position.y
        self.setpoint_msg.position.z
        self.setpoint_msg.velocity.x
        self.setpoint_msg.velocity.y
        self.setpoint_msg.velocity.z
        self.setpoint_msg.yaw
        self.setpoint_msg.yaw_rate

    These are all in SI units (m, m/s, rad, rad/s). The command-sending 
    has been abstracted away into 

        self.set_velocity_command(vx, vy, vz, yr)
        self.set_position_command(px, py, pz, yaw)
    """
    def __init__(self):
        super(ControllerNode, self).__init__()

        rospy.init_node('controller')
        
        self.thread_ready = False
        self.kill_thread = False
        self.ready_topics = {key: False for key in ['state', 'imu',
                             'pose', 'extended_state']}

        # Wait for services to become available
        service_timeout = 30
        try:
            rospy.wait_for_service('~/mavros/set_mode', service_timeout)
            rospy.wait_for_service('~/mavros/cmd/arming', service_timeout)
            self._services_online = True
        except rospy.ROSException:
            self._services_online = False

        # Create service proxies
        self.set_mode_srv = rospy.ServiceProxy('~/mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('~/mavros/cmd/arming', CommandBool)

        # Subscribers
        self.state_sub = rospy.Subscriber('~/mavros/state', State,
                                          self.cb_mavros_state)
        self.imu_data_sub = rospy.Subscriber('~/mavros/imu/data', Imu,
                                               self.cb_mavros_imu)
        self.ext_state_data_sub = rospy.Subscriber('~/mavros/extended_state', ExtendedState,
                                               self.cb_mavros_extended_state)
        self.pose_sub = rospy.Subscriber('~/mavros/local_position/pose', PoseStamped,
                                               self.cb_mavros_pose)

        # Publisher
        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        self.setpoint_msg = PositionTarget()
        self.setpoint_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.set_position_command(0,0,0,0)
        self.state = State()
        self.pose = PoseStamped()
        self.extended_state = ExtendedState()

        # Send setpoints in seperate thread to better prevent failsafe.
        # If PX4 does not receive a constant stream of setpoints, it will
        # activate a failsafe.
        self.setpoint_thread = Thread(target=self.send_setpoint, args=())
        self.setpoint_thread.daemon = True
        self.setpoint_thread.start()

    def cb_mavros_pose(self, pose_msg):
        self.pose = pose_msg
        self.ready_topics['pose'] = True 

    def cb_mavros_state(self, state_msg):
        self.state = state_msg
        self.ready_topics['state'] = True 

    def cb_mavros_extended_state(self, extended_state_msg):
        self.extended_state = extended_state_msg
        self.ready_topics['extended_state'] = True 
    
    def cb_mavros_imu(self, imu_msg):
        self.ready_topics['imu'] = True

    def send_setpoint(self):
        """
        This function runs in a seperate thread, sending setpoint commands to 
        the FCU at 10Hz.

        This is performed by reading the class internal variable `setpoint_msg`,
        and continuously sending its value to the FCU.

        This function is called once.
        """
        rate = rospy.Rate(10)  # Hz
        self.setpoint_msg.header = Header()
        self.setpoint_msg.header.frame_id = "base_footprint"
        self.thread_ready = True # Thread is ready.
        while not rospy.is_shutdown():
            if not self.kill_thread:
                self.setpoint_msg.header.stamp = rospy.Time.now()
                self.setpoint_pub.publish(self.setpoint_msg)
                try:  # prevent garbage in console output when thread is killed
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass
            else:
                rospy.logwarn_once('Halted sending commands to FCU.')
            
    def preflight_check(self, timeout = 30):
        # Checks to implement?
        # - Proper communication with PX4
        # - All other nodes are healthy
        #   - Will need a list of critical nodes
        # - Proper interface with user?
        # - Motors work
        # - All sensors work
        # - Mission is known somehow
        #   - Just take off and hover? (can default to this)
        #   - Source seeking? Some other node can actually decide the command.
        # This is quite the implementation. We should have a seperate diagnostics node.

        loop_freq = 5
        rate = rospy.Rate(loop_freq)
        preflight_passed = False
        for i in xrange(loop_freq * timeout):    
            # Checks:
            # - thread is ready
            # - services online
            # - getting data from all subscribed topics
            if self.thread_ready and all(self.ready_topics.values()) and self._services_online:
                preflight_passed = True 
                rospy.loginfo("IFO_CORE: Preflight check passed.")
                break
            rate.sleep()

        if not preflight_passed:
            # TODO: add reason for failure
            rospy.logerr("IFO_CORE: Preflight check failed.")

        return preflight_passed

    def takeoff(self, target_altitude = 1, timeout = 20):
        """
        Arms the quadcopter, and increases altitude until the target altitude is
        reached. RELIES ON STATE ESTIMATE FEEDBACK.

        Simple proportional control is used.
        """
        preflight_success = self.preflight_check()
        mode_success = self.set_mode('OFFBOARD',3)
        arm_success = self.set_arm(True, 3)
        
        altitude_reached = False
        threshold = 0.2 # considered reached when within threshold meters of target altitude
        loop_freq = 50 # Hz
        rate = rospy.Rate(loop_freq)
        takeoff_successful = False
        for i in xrange(timeout * loop_freq):
            # Get altitude from state estimator
            altitude = self.pose.pose.position.z

            # Simple proportional control
            k_p = 1.5 # Gain
            u = k_p*(target_altitude - altitude)
            self.set_velocity_command(vz = u)

            if abs(target_altitude - altitude) < threshold:
                altitude_reached = True 
                rospy.loginfo('IFO_CORE: Takeoff successful, altitude reached.')
                self.set_velocity_command(vz = 0)                
                break   

            rate.sleep()
   
    def land(self):
        """ 
        Lands the quadcopter where it is.
        """
        self.set_velocity_command(vz = -0.6) # Send downwards velocity command.
        self.set_mode('AUTO.LAND',5) # Simulanteously activate automatic landing.
        is_landed = self.wait_for_landed_state(10) # This is failing often.
        if is_landed:
            self.set_arm(False, 5) # Landed state required for disarming.
        else:
            rospy.logwarn('Automatic landing failed. Sending downwards velocity command.')
            self.set_velocity_command(vz = -1.5)
            is_landed = self.wait_for_landed_state(3)
            if is_landed:
                self.set_arm(False, 5)
            else: 
                rospy.logwarn('Landing state still not detected. Killing.')
                self.kill_thread = True
    
    def set_mode(self, mode, timeout):
        """ 
        Sets the FCU flight mode.

        mode: PX4 mode string
        timeout(int): seconds
        """
        rospy.loginfo("IFO_CORE: setting FCU mode: {0}".format(mode))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        set_success = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                set_success = True
                rospy.loginfo("IFO_CORE: set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("IFO_CORE: failed to send mode command.")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()
        return set_success

    def set_arm(self, arm, timeout):
        """
        Sets the arming mode of the FCU.
        arm: True to arm or False to disarm
        timeout(int): seconds
        """
        rospy.loginfo("IFO_CORE: setting FCU arm: {0}".format(arm))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        set_success = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo("IFO_CORE: set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                set_success = True
                break     
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("IFO_CORE: arming failed.")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()
        return set_success

    def set_velocity_command(self, vx = 0, vy = 0, vz = 0, yr = 0):
        """ 
        Sets the velocity command to the internal setpoint_msg variable.
        A seperate thread sends the setpoint_msg to the PX4 controller.
        """
        self.setpoint_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_msg.type_mask = PositionTarget.IGNORE_PX \
                                    + PositionTarget.IGNORE_PY \
                                    + PositionTarget.IGNORE_PZ \
                                    + 0*PositionTarget.IGNORE_VX \
                                    + 0*PositionTarget.IGNORE_VY \
                                    + 0*PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX \
                                    + PositionTarget.IGNORE_AFY \
                                    + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.IGNORE_YAW \
                                    + 0*PositionTarget.IGNORE_YAW_RATE
        self.setpoint_msg.velocity.x = vx
        self.setpoint_msg.velocity.y = vy
        self.setpoint_msg.velocity.z = vz
        self.setpoint_msg.yaw_rate = yr

    def set_position_command(self, px = 0, py = 0, pz = 0, yaw = 0):
        """ 
        Sets the position command to the internal setpoint_msg variable.
        A seperate thread sends the setpoint_msg to the PX4 controller.
        """
        self.setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_msg.type_mask = PositionTarget.IGNORE_VX \
                                    + PositionTarget.IGNORE_VY \
                                    + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX \
                                    + PositionTarget.IGNORE_AFY \
                                    + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.IGNORE_YAW_RATE
        if px is None:
            px = self.pose.pose.position.x            

        if py is None:
            py = self.pose.pose.position.y
            
        if pz is None:
            pz = self.pose.pose.position.z
            
        if yaw is None:
            self.setpoint_msg.type_mask += PositionTarget.IGNORE_YAW
            yaw = 0
            

        self.setpoint_msg.position.x = px
        self.setpoint_msg.position.y = py
        self.setpoint_msg.position.z = pz
        self.setpoint_msg.yaw = yaw

    def wait_for_landed_state(self, timeout):
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == 1: # 1 for landed, 0 for not landed
                landed_state_confirmed = True
                rospy.loginfo("IFO_CORE: landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            rate.sleep()
        return landed_state_confirmed


if __name__ == "__main__":
    controller = ControllerNode()
    controller.takeoff()
    controller.set_position_command(px = 1, py = 1, pz = 1, yaw = 0)
    sleep(5)
    controller.set_position_command(px = 1, py = -1, pz = 1, yaw = 3.14159/2)
    sleep(5)
    controller.set_position_command(px = -1, py = -1, pz = 1, yaw = 3.14159)
    sleep(5)
    controller.set_position_command(px = -1, py = 1, pz = 1, yaw = -3.14159/2)
    sleep(5)
    controller.land()
    rospy.spin()

