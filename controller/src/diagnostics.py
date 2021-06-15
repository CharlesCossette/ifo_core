
import rospy
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from std_msgs.msg import Header
from threading import Thread
from time import sleep