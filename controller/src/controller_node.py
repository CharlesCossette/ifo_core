#!/usr/bin/env python2

import rospy
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from std_msgs.msg import Header
from threading import Thread
from time import sleep

class ControllerNode(object):
    """
    This class starts a thread which continuously publishes its own 
    `setpoint_msg` property on the /mavros/setpoint_raw/local topic.

    As is, the `setpoint_msg` requires the following values:

        self.setpoint_msg.velocity.x
        self.setpoint_msg.velocity.y
        self.setpoint_msg.velocity.z
        self.setpoint_msg.yaw_rate

    These are all in SI units (m/s, )
    """
    def __init__(self):
        super(ControllerNode, self).__init__()

        rospy.init_node('controller')
        
        self.ready_thread = False
        self.ready_topics = {key: False for key in ['state', 'imu']}

        # Wait for services to become available
        service_timeout = 30
        try:
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            self._services_online = True
        except rospy.ROSException:
            self._services_online = False

        # Create service proxies
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # Subscribers
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.cb_mavros_state)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu,
                                               self.cb_mavros_imu)

        # Publisher
        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        self.setpoint_msg = PositionTarget()
        self.setpoint_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.set_position_command(0,0,0,0)
        self.state = State()

        # Send setpoints in seperate thread to better prevent failsafe
        self.setpoint_thread = Thread(target=self.send_setpoint, args=())
        self.setpoint_thread.daemon = True
        self.setpoint_thread.start()


    def cb_mavros_state(self, state_msg):
        self.state = state_msg
        self.ready_topics['state'] = True 
    
    def cb_mavros_imu(self, imu_msg):
        self.ready_topics['imu'] = True

    def send_setpoint(self):
        """
        This function runs in a seperate thread, sending setpoint commands to 
        the FCU at 10Hz.

        This is performed by reading the class internal variable `setpoint_msg`,
        and continuously sending its value to the FCU.
        """
        rate = rospy.Rate(10)  # Hz
        self.setpoint_msg.header = Header()
        self.setpoint_msg.header.frame_id = "base_footprint"
        self.ready_thread = True # Thread is ready.
        while not rospy.is_shutdown():
            self.setpoint_msg.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(self.setpoint_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def start(self):
        preflight_passed = self.preflight_check()
        if preflight_passed:
            rospy.loginfo("IFO_CORE: Preflight check passed.")
            self.takeoff()
        else:
            rospy.loginfo("Preflight check failed.")
            

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
        #   
        rate = rospy.Rate(0.5)
        preflight_passed = False
        start_time = rospy.get_time()
        iter_failsafe = 0
        while start_time == 0.0 and iter_failsafe < 100:
            start_time = rospy.get_time()
            sleep(0.01)
            iter_failsafe += 1

        while not preflight_passed and rospy.get_time() - start_time < timeout:
            if self.ready_thread and \
               all(self.ready_topics.values()):
                preflight_passed = True 
            rate.sleep()
        return preflight_passed

    def takeoff(self, altitude = 1):
        # TODO. Add feedback
        self.set_mode('OFFBOARD',3)
        self.set_arm(True, 3)
        self.set_position_command(pz = 1)
        sleep(15)
        self.set_position_command(px = 1, pz =1, yaw = 3.14159/2)
        sleep(15)
        self.set_position_command(pz =1)
        sleep(10)
        self.land()
   
    def land(self):
        self.set_velocity_command(vz = -1)    
    
    def set_mode(self, mode, timeout):
        """ 
        Sets the FCU flight mode.

        mode: PX4 mode string
        timeout(int): seconds
        """
        rospy.loginfo("setting FCU mode: {0}".format(mode))
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
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        set_success = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo("ifo_core: set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                set_success = True
                break     
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("ifo_core: arming failed.")
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
        self.setpoint_msg.type_mask = 0*PositionTarget.IGNORE_PX \
                                    + 0*PositionTarget.IGNORE_PY \
                                    + 0*PositionTarget.IGNORE_PZ \
                                    + PositionTarget.IGNORE_VX \
                                    + PositionTarget.IGNORE_VY \
                                    + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX \
                                    + PositionTarget.IGNORE_AFY \
                                    + PositionTarget.IGNORE_AFZ \
                                    + 0*PositionTarget.IGNORE_YAW \
                                    + PositionTarget.IGNORE_YAW_RATE
        self.setpoint_msg.position.x = px
        self.setpoint_msg.position.y = py
        self.setpoint_msg.position.z = pz
        self.setpoint_msg.yaw = yaw


if __name__ == "__main__":
    node = ControllerNode()
    node.start()
    rospy.spin()