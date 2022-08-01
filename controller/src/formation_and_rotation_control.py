import math
import numpy as np
import scipy.linalg
import pylie

class FormationAndRotationControl:

    def update_dcm(C_ia, A, omega_ia_i,dt):
        F=[]
        for i in range(len(A[0])):
            # Update DCM's
            C_ai = C_ia[i].T @ pylie.SO3.exp(pylie.SO3.wedge(omega_ia_i[i]*dt))
            C_ia[i] = C_ai.T
            # Update local reference frames
            F.append(C_ia[i])
        return C_ia,F

    def formation_control_gazebo(i,A,r_ij_rel_i, r_ij_rel_des_i):

        # 2*error norms. Errors have no directions
        r_ij_star = {}      # ||rij||**2 - ||rij_des||**2
        for j in range(len(A[0])):
            if(A[i,j]== 1):
                r_ij_star[i,j] = np.linalg.norm(r_ij_rel_i[i,j])**2 - np.linalg.norm(r_ij_rel_des_i[i,j])**2

        # kp
        kp = 1
        kc = 100
        u_iw_i = np.zeros([1,3])
        for j in range(len(A[0])):
            if (A[i,j]== 1):
                u_iw_i = u_iw_i+(-1/2*kp*(r_ij_rel_i[i,j]*r_ij_star[i,j]))

        # Bound the control effort based on magnitude
        u_iw_i = kc*(u_iw_i/np.linalg.norm(u_iw_i)) if np.linalg.norm(u_iw_i) > kc else u_iw_i
            
        return u_iw_i

    def apply_formation_control_effort(u_iw_i, C_ia, r_a, dt):
        # apply the control effort in local reference frame
        r_iw_a = {}
        r_iw_i = {}
        for i in range(len(u_iw_i)):
            # update position of each point in Fa
            r_iw_a[i]= r_a[i]
        for i in range(len(u_iw_i)):
            r_iw_i[i] = C_ia [i] @ r_iw_a  [i].reshape(-1,1)
            r_iw_i[i] = r_iw_i[i] + u_iw_i  [i].reshape(-1,1)*dt
            r_iw_a[i] = C_ia [i].T @ r_iw_i[i].reshape(-1,1)
        r_a = np.array(list(r_iw_a.values())).reshape(-1,3)
        return r_a
    
    def rotation_control(i, A, C_ia, C_ij_star):
        C_ij={}
        for j in range(len(A[0])):
            if (j!=i): # Needs undirected graph
                C_ij[i,j] = C_ia[i] @ C_ia[j].T
        
        kj = 1
        # update w's for attitude control
        omega_ia_i = np.zeros([3,1])
        for j in range(len(A[0])):
            if (j!=i):
                omega_ia_i = omega_ia_i + (kj*(pylie.SO3.vee(pylie.SO3.log(C_ij[i,j] @ C_ij_star[i,j].T))))
        return omega_ia_i

    def rotation_control_with_Cij(i, A, C_ij, C_ij_star):
        
        kj = 100
        # update w's for attitude control
        omega_ia_i = np.zeros([3,1])
        for j in range(len(A[0])):
            if (j!=i): # Needs connected, undirected graph
                omega_ia_i = omega_ia_i + (kj*(pylie.SO3.vee(pylie.SO3.log(C_ij[i,j] @ C_ij_star[i,j].T))))
        return omega_ia_i