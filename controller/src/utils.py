import numpy as np
import pylie
    
def relative_dcm(no_of_agents, C_ia):
    C_ij_rel={}
    for i in range(no_of_agents):
        for j in range(no_of_agents):
            if(j!=i):
                C_ij_rel[i,j] = C_ia[i] @ C_ia[j].T
    return C_ij_rel

def relative_positions_in_local_frame(no_of_agents,r_a,C_ia):
    r_ij_rel_g = {}       # relative displacement between i and j in Fa
    r_ij_rel_i = {}       # relative displacement between i and j in Fi
    
    # Update relative positions in global and local reference frames
    for i in range(no_of_agents):
        for j in range(no_of_agents):
            if (j!=i):
                r_ij_rel_g[i,j] = r_a[i] - r_a[j]
                r_ij_rel_i[i,j] = C_ia[i] @ r_ij_rel_g[i,j]
    return r_ij_rel_i