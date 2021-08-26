#!/usr/bin/env python2

import rospy
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, ExtendedState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
from threading import Thread
from time import sleep
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from controller.srv import ReachWaypointList, ReachWaypointListResponse
from controller.msg import Waypoint
import yaml
import rospkg
from ifo_common.ifo_node import IfoNode

class ControllerNode(IfoNode):
    """
    This class starts a thread which continuously publishes its own 
    `setpoint_msg` property on the /mavros/setpoint_raw/local topic.

    The command-sending has been abstracted away into 

        self.set_velocity_command(vx, vy, vz, yr)
        self.set_position_command(px, py, pz, yaw)
    """
    def __init__(self):
        rospy.init_node('controller')
        super(ControllerNode, self).__init__()

        
        self.thread_ready = False
        self.kill_thread = False
        self.ready_topics = {key: False for key in ['state', 'imu',
                             'pose', 'extended_state']}
        self.report_diagnostics(level=1, message='Initializing.')

        # Wait for services to become available
        service_timeout = 20
        try:
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            self._services_online = True
        except rospy.ROSException:
            rospy.logwarn('IFO CORE: Controller required services not online.')
            self._services_online = False

        # Create service proxies
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # Subscribers
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.cb_mavros_state)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu,
                                               self.cb_mavros_imu)
        self.ext_state_data_sub = rospy.Subscriber('mavros/extended_state', ExtendedState,
                                               self.cb_mavros_extended_state)
        self.pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                                               self.cb_mavros_pose)

        # Publishers
        self.setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=1
        )
        
        # Services
        self.reach_waypoint_list_srv = rospy.Service(
            'controller/reach_waypoint_list', ReachWaypointList, self.cb_reach_waypoint_list
        )

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

        self.report_diagnostics(level=0, message='Ready.')
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
                takeoff_successful = True 
                rospy.loginfo('IFO_CORE: Takeoff successful, altitude reached.')
                self.set_velocity_command(vz = 0)             
                break

            rate.sleep()
        return takeoff_successful 
        
    def hold_altitude(self, target_altitude = None, duration = 1):
        loop_freq = 50 # Hz
        rate = rospy.Rate(loop_freq)
        altitude = self.pose.pose.position.z
        if target_altitude is None:
            target_altitude = altitude
            
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < duration:
            # Get altitude from state estimator
            altitude = self.pose.pose.position.z

            # Simple proportional control
            k_p = 1.5 # Gain
            u = k_p*(target_altitude - altitude)
            self.set_velocity_command(vz = u)
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

    def load_waypoint_list(self, filename = None):
        """ 
        Reads the yaml file located at /controller/config/waypoint_list.yaml
        """
        rp = rospkg.RosPack()
        if filename is None:            
            path = rp.get_path('controller')
            filename = path + '/config/waypoint_list.yaml'

        with open(filename) as file:
            waypoint_list = yaml.load(file, Loader=yaml.FullLoader)

        who_am_i = rospy.get_namespace()
        if who_am_i == '/':
            rospy.loginfo('Controller node not launched in a namespace. Assuming ifo001.')
            who_am_i = 'ifo001'
        else:
            who_am_i = who_am_i.strip('/')

        if who_am_i in waypoint_list:
            temp = waypoint_list[who_am_i]
            my_waypoint_list = []
            for key, value in temp.iteritems():
                wp = {key: value[key] for key in ['x','y','z','yaw']}
                my_waypoint_list.append((value['time'], wp))

            # Sort by time.
            my_waypoint_list = sorted(my_waypoint_list, key=lambda x: x[0])
            self.waypoint_list = my_waypoint_list
            print(self.waypoint_list)
        else:
            rospy.logwarn('Cannot find ' + who_am_i + ' in waypoint list.')

    def reach_waypoint_list(self, waypoint_list):

        self.wait_for_nodes('mocap_forwarder')
        self.report_diagnostics(level=0, message='Normal.')
        self.takeoff()
        start_time = rospy.get_time()
        first_waypoint = True
        for wp in waypoint_list:
            t_waypoint = wp.time
            while rospy.get_time() - start_time < t_waypoint:
                if first_waypoint:
                    self.hold_altitude(target_altitude=1,duration=0.01)
                else:
                    rospy.sleep(0.01)
            print(wp)
            self.set_position_command(px=wp.x, py=wp.y, pz=wp.z, yaw=wp.yaw)
            first_waypoint = False

        rospy.sleep(5)
        self.land()
    
    def cb_reach_waypoint_list(self, request):
        self.reach_waypoint_list(request.waypoint_list)
        response = ReachWaypointListResponse()
        response.success = True
        return response

    def reach_waypoint(self, waypoint):
        pass

if __name__ == "__main__":
    controller = ControllerNode()
    rospy.spin()

