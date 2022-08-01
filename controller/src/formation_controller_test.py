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
import util
import pylie
from controller.msg import RelPosition, RelPositionList, RelOrientation, RelOrientationList


class GetRelativePosition():

    def __init__(self, rel_position_topic):

        self.rel_position = rospy.Subscriber(rel_position_topic, RelPositionList, self.callback_position)
        self._received_first_message = False

    def callback_position(self, rel_position):
        self._received_first_message = True
        self.relative_positions = rel_position.relative_position
        a = 2

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
    
    rel_position_topics = ["/ifo001/r_ij_rel_i"]
                        #    "/ifo002/r_ij_rel_i",
                        #    "/ifo003/r_ij_rel_i"]
    
    rel_orientation_topics = ["/ifo001/C_ij"]
                            #   "/ifo002/C_ij",
                            #   "/ifo003/C_ij"]

    # lapacian matrix
    L = np.array([[2,-1,-1],
                  [-1,2,-1],
                  [-1,-1,2]])
    # find D and A
    D = np.diag(np.diag(L))
    A = D - L

    position_receivers = [GetRelativePosition(topic) for topic in rel_position_topics]
    orientation_receivers = [GetRelativeOrientation(topic) for topic in rel_orientation_topics]

    position_receiver = GetRelativePosition(rel_position_topics)
    orientation_receiver = GetRelativeOrientation(rel_orientation_topics)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        r_ia_w = []
        C_ia = []
        for position_receiver in position_receivers:
            r_ij_rel_i = position_receiver.get_rel_position()
        for orientation_receiver in orientation_receivers:
            C_ij_rel = orientation_receiver.get_rel_orientation()
            if r_ij_rel_i is not None:
                a = 2
                b= 2
                # r_ia_w.append(pose[0])
                # C_ia.append(pose[1])

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
