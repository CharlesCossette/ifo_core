#!/usr/bin/env python3
from ast import Global
from binascii import rlecode_hqx
from socket import IPV6_JOIN_GROUP
from pyparsing import java_style_comment
import rospy
import numpy as np
import pylie
from controller.msg import RelPositionList, RelOrientationList
from mavros_controller import ControllerNode
from formation_and_rotation_control import formation_control, rotation_control
import utils
import relative_pose_update
np.random.seed(1234)



class RelativePositionGetter():

    def __init__(self, rel_position_topic):

        self.rel_position = rospy.Subscriber(rel_position_topic, RelPositionList, self.callback_position)
        self._received_first_message = False

    def callback_position(self, rel_position):
        self._received_first_message = True
        self.relative_positions = rel_position.relative_position

    def get_rel_position(self):
        if self._received_first_message:
            r_ij_rel_i = {}
            for relative_position in self.relative_positions:
                i = relative_position.agent_i.data
                j = relative_position.agent_j.data
                x = relative_position.r_ij_rel_i.x
                y = relative_position.r_ij_rel_i.y
                z = relative_position.r_ij_rel_i.z
                r_ij_rel_i[i,j]=np.array([x,y,z])
            return r_ij_rel_i
        else:
            return None

class RelativeOrientationGetter():

    def __init__(self, rel_orientation_topic):

        self.rel_orientation = rospy.Subscriber(rel_orientation_topic, RelOrientationList, self.callback_orientation)
        self._received_first_message = False

    def callback_orientation(self, rel_orientation):
        self._received_first_message = True
        self.relative_orientations = rel_orientation.relative_orientation

    def get_rel_orientation(self):
        if self._received_first_message:
            C_ij_rel = {}
            for relative_orientation in self.relative_orientations:
                i = relative_orientation.agent_i.data
                j = relative_orientation.agent_j.data
                w = relative_orientation.C_ij_rel.w
                x = relative_orientation.C_ij_rel.x
                y = relative_orientation.C_ij_rel.y
                z = relative_orientation.C_ij_rel.z
                Q_ij = np.array([w,x,y,z])
                C_ij_rel[i,j] = pylie.SO3.from_quat(Q_ij,order="wxyz")
            return C_ij_rel
        else:
            return None

if __name__ == "__main__":
    control=ControllerNode()
    
    agent_name = rospy.get_namespace()
    control.takeoff()
    rospy.sleep(5)
    if agent_name =='/ifo001/':
        agent_i = 0
        control.set_position_command(0, 3, 10, 0)
    elif agent_name =='/ifo002/':
        agent_i = 1
        control.set_position_command(3, 0, 10, 0)
    elif agent_name =='/ifo003/':
        agent_i = 2
        control.set_position_command(-3, 0, 10, 0)
    rospy.sleep(20)

    ##### Problem setup #####
    topics = ["/ifo001/vrpn_client_node/ifo001/pose",
              "/ifo002/vrpn_client_node/ifo002/pose",
              "/ifo003/vrpn_client_node/ifo003/pose"]
    
    no_of_agents = len(topics)
    
    # desired position in Fa
    r_des_a = np.array([[0,3,10],
                        [3,0,10],
                        [-3,0,10]])

    # desired C_ia
    C_ia_star = []
    for i in range(no_of_agents):
        C_ia_star.append(pylie.SO3.random())

    # Angular velocities, i is the individual body frame number
    omega_ia_i    = []
    for i in range(no_of_agents):
        omega_ia_i.append(np.block([0,0,0]))
    ###### User Inputs End Here ######

    receivers = [relative_pose_update.GetGlobalPose(topic) for topic in topics]
    C_ia  = []
    rate = rospy.Rate(50)
    for recv in receivers:
        pose = None 
        while pose is None and not rospy.is_shutdown():
            pose = recv.get_global_pose() 
            rate.sleep()

        C_ia.append(pose[1])

    # Find C_ij_star's
    C_ij_star_rel = utils.relative_dcm(no_of_agents, C_ia_star)

    # find desired distances in global frame and individual body frames
    r_ij_rel_des_i = utils.relative_positions_in_local_frame(no_of_agents,r_des_a,C_ia)
    ##### Problem setup ends here #####
    
    # Subsribe to relative position nodes
    rel_position_topic = "r_ij_rel_i"
    rel_orientation_topic = "C_ij"
    rel_position_receiver = RelativePositionGetter(rel_position_topic)
    rel_orientation_receiver = RelativeOrientationGetter(rel_orientation_topic)

    # Apply conrol law
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        r_ij_rel_i = rel_position_receiver.get_rel_position()
        C_ij_rel = rel_orientation_receiver.get_rel_orientation()
        if r_ij_rel_i and C_ij_rel is not None:
            u_iw_i = formation_control(agent_i, no_of_agents, r_ij_rel_i, r_ij_rel_des_i).reshape(-1,1)
            omega_ia_i = rotation_control(agent_i, no_of_agents, C_ij_rel, C_ij_star_rel)
            control.set_velocity_command(u_iw_i[0],u_iw_i[1],u_iw_i[2],omega_ia_i[2]) #(u_iw_i[0],u_iw_i[1],0,0)
        rate.sleep()