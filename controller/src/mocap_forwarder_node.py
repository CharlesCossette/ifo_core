import rospy 
from mavros_msgs.srv import ParamGet, ParamSet
from mavros_msgs.msg import ParamValue
from geometry_msgs.msg import PoseStamped

class MocapForwarder(object):
    def __init__(self):
        super(MocapForwarder, self).__init__()

        # Wait for services to become available
        service_timeout = 30
        try:
            rospy.wait_for_service('~/mavros/param/set', service_timeout)
            rospy.wait_for_service('~/mavros/param/get', service_timeout)
            self._services_online = True
        except rospy.ROSException:
            self._services_online = False
        
        rospy.init_node('mocap_forwarder')
        self.set_param_srv = rospy.ServiceProxy('~/mavros/param/set', ParamSet)
        self.get_param_srv = rospy.ServiceProxy('~/mavros/param/get', ParamGet)
        self.pose_sub = rospy.Subscriber('/vrpn_client_node/ifo/pose', PoseStamped,
                                         self.cb_mocap_pose)
        self.pose_pub = rospy.Publisher('~/mavros/vision_pose/pose',PoseStamped)



    def cb_mocap_pose(self, pose_msg):
        self.pose = pose_msg

    def set_parameter(self, param_id, value, timeout = 10):
        # Sets an FCU parameter and verifies that it has been done. 
        rospy.loginfo('IFO_CORE: Setting FCU parameter ' + param_id + ' to: ' + str(value))
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        set_success = False
        param_value = ParamValue()
        if isinstance(value, int):
            param_value.integer = value
        else:
            param_value.real = value

        for i in xrange(timeout * loop_freq):
            resp = self.set_param_srv(param_id, param_value)
            value_check, _ = self.get_parameter(param_id) 
            if isinstance(value, int):
                value_check = value_check.integer
            else:
                value_check = value_check.real
            if value_check == value:
                rospy.loginfo('IFO_CORE: Setting FCU parameter ' + param_id + ': SUCCESS')
                set_success = True
                break
            rate.sleep()
        if not set_success:
            rospy.loginfo('IFO_CORE: Setting FCU parameter ' + param_id + ': FAILED')
        return set_success

    def get_parameter(self, param_id, timeout = 10):
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            resp = self.get_param_srv(param_id)
            if resp.success:
                break
            rate.sleep()
        return resp.value, resp.success

    def start(self):
        # Set the right PX4 parameters for when using mocap
        self.set_parameter('EKF2_HGT_MODE', 3)
        self.set_parameter('EKF2_AID_MASK', 0b000011000)
        self.set_parameter('EKF2_EV_DELAY', 0.0)
        self.set_parameter('EKF2_EV_POS_X', 0.0)
        self.set_parameter('EKF2_EV_POS_Y', 0.0)
        self.set_parameter('EKF2_EV_POS_Z', 0.0)  
        self.set_parameter('MAV_ODOM_LP', 1)   
        loop_freq = 40
        rate = rospy.Rate(40)
        while True:
            self.pose_pub.publish(self.pose)
            rate.sleep()

if __name__ == "__main__":
    node = MocapForwarder()
    sleep(5)
    node.start()