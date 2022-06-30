#!/usr/bin/env python2
from mavros_controller import ControllerNode
import rospy 

cont = ControllerNode()
cont.takeoff()
cont.set_position_command(2, 1, 1, 0)
rospy.sleep(5)
cont.set_position_command(2, -3, 1, 0.4)
rospy.sleep(5)
cont.set_position_command(0, 0, 1, 0)
rospy.sleep(5) 
cont.land()