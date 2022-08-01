# !/usr/bin/env python2
# from mavros_controller import ControllerNode
from ast import Global
from binascii import rlecode_hqx
from socket import IPV6_JOIN_GROUP
from pandas import Float64Index
from pyparsing import java_style_comment
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64MultiArray
import numpy as np
import pylie
from controller.msg import RelPosition, RelPositionList, RelOrientation, RelOrientationList
from mavros_controller import ControllerNode
import formation_and_rotation_control
import utils
import relative_pose_update
np.random.seed(1234)

global_pose = relative_pose_update
func = formation_and_rotation_control.FormationAndRotationControl
util = utils.Utilities

class GetRelativePosition():

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

class GetRelativeOrientation():

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

    rospy.init_node('formation_controller')
    agent_i = 0 # agent = 0 represents ifo001
    rel_position_topic = "/ifo001/r_ij_rel_i"
    rel_orientation_topic = "/ifo001/C_ij"
    rel_position_receiver = GetRelativePosition(rel_position_topic)
    rel_orientation_receiver = GetRelativeOrientation(rel_orientation_topic)
    
    
    
    ##### Problem setup #####
    topics = ["/ifo001/vrpn_client_node/ifo001/pose",
              "/ifo002/vrpn_client_node/ifo002/pose",
              "/ifo003/vrpn_client_node/ifo003/pose"]

    # lapacian matrix
    L = np.array([[2,-1,-1],
                  [-1,2,-1],
                  [-1,-1,2]])
    # find D and A
    D = np.diag(np.diag(L))
    A = D - L

    # desired position in Fa
    r_des_a = np.array([[1,7,-1],
                        [7,7,-4],
                        [7,1, 4]])
    
    # desired C_ia
    C_ia_star = []
    for i in range(len(L[0])):
        C_ia_star.append(pylie.SO3.random())

    # Angular velocities, i is the individual body frame number
    omega_ia_i    = []
    for i in range(len(L)):
        omega_ia_i.append(np.random.rand(3,1))

    ###### User Inputs End Here ######

    receivers = [global_pose.GetGlobalPose(topic) for topic in topics]
    C_ia = []
    while not rospy.is_shutdown():
        for receiver in receivers:
            pose = receiver.get_global_pose()
            if pose is not None:
                C_ia.append(pose[1])
        if (len(C_ia) == np.size(topics)):
            break

    # Find C_ij_star's
    C_ij_star_rel = util.relative_dcm(A, C_ia_star)

    # find desired distances in global frame and individual body frames
    r_ij_rel_des_i = util.relative_positions_in_local_frame(A,r_des_a,C_ia)

    ##### Problem setup ends here #####


    control = ControllerNode()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        r_ij_rel_i = rel_position_receiver.get_rel_position()
        C_ij_rel = rel_orientation_receiver.get_rel_orientation()
        if r_ij_rel_i and C_ij_rel is not None:
            u_iw_i = func.formation_control_gazebo(agent_i, A, r_ij_rel_i, r_ij_rel_des_i)
            omega_ia_i = func.rotation_control_with_Cij(agent_i, A, C_ij_rel, C_ij_star_rel)
            control.set_velocity_command(u_iw_i[0],u_iw_i[1],u_iw_i[2],omega_ia_i[3])
            print(u_iw_i)
        rate.sleep()

    




    # # obj=GlobalPose()
    # # obj.sub(i)
    # # for m in range(np.size(i)):
    # rospy.sleep(1)
    # pose0 = GlobalPose(i[0]).global_pose()
    # pose1 = GlobalPose(i[1]).global_pose()
    # pose2 = GlobalPose(i[2]).global_pose()
    # # r_ia_w.append(pose0[0])
    # # C_ia.append(pose0[1])

    # # r_ia_w, C_ia = pose.global_position()
    # print(r_ia_w)
    # print("\n")
    # print(C_ia)
    # print("\n")
    # rospy.spin()

    # rel_pose = RelativePose(A, r_ia_w, C_ia)
    # r_ij_rel_i = rel_pose.relative_positions_in_local_frame()
    # C_ij_rel = rel_pose.relative_dcm()

    # print(r_ij_rel_i)
    # print(node.traj_pose)


                        # for i in range(len(A)):
                    #     rel_orientation_list = RelOrientationList()
                    #     for C in C_ij_rel:
                    #         if (i==C[0]):
                    #             rel_orientation = RelOrientation()
                    #             rel_orientation.agent_i.data=C[0]
                    #             rel_orientation.agent_j.data=C[1]
                                
                    #             Q_ij = pylie.SO3.to_quat(C_ij_rel[C],order="wxyz")
                    #             rel_orientation.C_ij_rel.w=Q_ij[0]
                    #             rel_orientation.C_ij_rel.x=Q_ij[1]
                    #             rel_orientation.C_ij_rel.y=Q_ij[2]
                    #             rel_orientation.C_ij_rel.z=Q_ij[3]
                    #             rel_orientation_list.relative_orientation.append(rel_orientation)
                                
                    #     publish_rel_orientation = PublishRelativeOrientation(rel_orientation_topics[i])
                    #     publish_rel_orientation.pub.publish(rel_orientation_list)
