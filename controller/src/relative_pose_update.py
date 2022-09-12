#!/usr/bin/env python3
# from mavros_controller import ControllerNode
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import pylie
from controller.msg import RelPosition, RelPositionList, RelOrientation, RelOrientationList


class GlobalPoseGetter():

    def __init__(self, topic):

        self.topic = topic
        self.traj_pose = rospy.Subscriber(self.topic, PoseStamped, self.callback_pose)
        self._received_first_message = False
        
    def callback_pose(self, pose):
        self._received_first_message = True
        self.pose = pose.pose.position
        self.orien = pose.pose.orientation


    def get_global_pose(self):
        if self._received_first_message:
            r_ia_w = np.array([self.pose.x,self.pose.y,self.pose.z])

            Q_ia   = np.array([self.orien.w, self.orien.x, self.orien.y, self.orien.z])
            C_ia   = pylie.SO3.from_quat(Q_ia).T
            return r_ia_w, C_ia
        else:
            return None

class ComputeRelativePose():
    
    def __init__(self, no_of_agents, r_ia_w, C_ia):
        self.no_of_agents = no_of_agents
        self.C_ia = C_ia
        self.r_ia_w = r_ia_w

    def relative_dcm(self):
        C_ij_rel={}
        for i in range(self.no_of_agents):
            for j in range(self.no_of_agents):
                if(j!=i):
                    C_ij_rel[i,j] = self.C_ia[i] @ self.C_ia[j].T
        return C_ij_rel
    
    def relative_positions_in_local_frame(self):
        r_ij_rel_g = {}       # relative displacement between i and j in Fa
        r_ij_rel_i = {}       # relative displacement between i and j in Fi
        
        # Update relative positions in global and local reference frames
        for i in range(self.no_of_agents):
            for j in range(self.no_of_agents):
                if (j!=i):
                    r_ij_rel_g[i,j] = self.r_ia_w[i] - self.r_ia_w[j]
                    r_ij_rel_i[i,j] = self.C_ia[i] @ r_ij_rel_g[i,j]
        return r_ij_rel_i

if __name__ == "__main__":
    
    # rostopics for relative pose
    rospy.init_node('ifos')
    topics = ["/ifo001/vrpn_client_node/ifo001/pose",
              "/ifo002/vrpn_client_node/ifo002/pose",
              "/ifo003/vrpn_client_node/ifo003/pose"]
    
    rel_position_topics = ["/ifo001/r_ij_rel_i",
                           "/ifo002/r_ij_rel_i",
                           "/ifo003/r_ij_rel_i"]
    
    rel_orientation_topics = ["/ifo001/C_ij",
                              "/ifo002/C_ij",
                              "/ifo003/C_ij"]
    
    no_of_agents = len(topics)

    receivers = [GlobalPoseGetter(topic) for topic in topics]
    pub_rel_position = []
    pub_rel_orientation = []
    for i in range(no_of_agents):
        pub_rel_position.append(rospy.Publisher(rel_position_topics[i], RelPositionList, queue_size=1))
        pub_rel_orientation.append(rospy.Publisher(rel_orientation_topics[i], RelOrientationList, queue_size=1))
    
    # allow time to receive messages
    rate = rospy.Rate(100)
    print('Node running...')
    while not rospy.is_shutdown():
        r_ia_w = []
        C_ia = []
        for receiver in receivers:
            pose = receiver.get_global_pose()
            if pose is not None:
                r_ia_w.append(pose[0])
                C_ia.append(pose[1])
        
        counter = len(r_ia_w)
        if counter == no_of_agents:        
            relative_pose = ComputeRelativePose(no_of_agents, r_ia_w, C_ia)
            C_ij_rel = relative_pose.relative_dcm()
            r_ij_rel_i = relative_pose.relative_positions_in_local_frame()
                
            # store and publish in custom message
            for i in range(no_of_agents):
                rel_orientation_list = RelOrientationList()
                rel_position_list = RelPositionList()

                for C in C_ij_rel:
                    if (i==C[0]):
                        rel_orientation = RelOrientation()
                        rel_orientation.agent_i.data=C[0]
                        rel_orientation.agent_j.data=C[1]
                        
                        Q_ij = pylie.SO3.to_quat(C_ij_rel[C],order="wxyz")
                        rel_orientation.C_ij_rel.w=Q_ij[0]
                        rel_orientation.C_ij_rel.x=Q_ij[1]
                        rel_orientation.C_ij_rel.y=Q_ij[2]
                        rel_orientation.C_ij_rel.z=Q_ij[3]
                        rel_orientation_list.relative_orientation.append(rel_orientation)
                
                for r in r_ij_rel_i:
                    if (i==r[0]):
                        rel_position = RelPosition()
                        rel_position.agent_i.data=r[0]
                        rel_position.agent_j.data=r[1]

                        rel_position.r_ij_rel_i.x=r_ij_rel_i[r][0]
                        rel_position.r_ij_rel_i.y=r_ij_rel_i[r][1]
                        rel_position.r_ij_rel_i.z=r_ij_rel_i[r][2]
                        rel_position_list.relative_position.append(rel_position)
                        
            
                pub_rel_position[i].publish(rel_position_list)
                pub_rel_orientation[i].publish(rel_orientation_list)

        rate.sleep()