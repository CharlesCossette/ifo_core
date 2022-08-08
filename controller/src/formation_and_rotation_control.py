import math
import numpy as np
import scipy.linalg
import pylie

def formation_control(i,no_of_agents,r_ij_rel_i, r_ij_rel_des_i):

    # 2*error norms. Errors have no directions
    r_ij_star = {}      # ||rij||**2 - ||rij_des||**2
    for j in range(no_of_agents):
        if(j!=i):
            r_ij_star[i,j] = np.linalg.norm(r_ij_rel_i[i,j])**2 - np.linalg.norm(r_ij_rel_des_i[i,j])**2

    # kp
    kp = 0.1
    kc = 100
    u_iw_i = np.zeros([1,3])
    for j in range(no_of_agents):
        if (j!=i):
            u_iw_i = u_iw_i+(-1/2*kp*(r_ij_rel_i[i,j]*r_ij_star[i,j]))

    # Bound the control effort based on magnitude
    u_iw_i = kc*(u_iw_i/np.linalg.norm(u_iw_i)) if np.linalg.norm(u_iw_i) > kc else u_iw_i
        
    return u_iw_i

def rotation_control(i, no_of_agents, C_ij, C_ij_star):
        
    kj = 100
    # update w's for attitude control
    omega_ia_i = np.zeros([3,1])
    for j in range(no_of_agents):
        if (j!=i): # Needs connected, undirected graph
            omega_ia_i = omega_ia_i + (kj*(pylie.SO3.vee(pylie.SO3.log(C_ij[i,j] @ C_ij_star[i,j].T))))
    return omega_ia_i