import numpy as np
import pylie

class Utilities:
    
    def relative_dcm(A, C_ia):
        C_ij_rel={}
        for i in range(len(A)):
            for j in range(len(A[0])):
                if(j!=i):
                    C_ij_rel[i,j] = C_ia[i] @ C_ia[j].T
        return C_ij_rel
    
    def relative_positions_in_local_frame(A,r_a,C_ia):
        r_ij_rel_g = {}       # relative displacement between i and j in Fa
        r_ij_rel_i = {}       # relative displacement between i and j in Fi
        
        # Update relative positions in global and local reference frames
        for i in range(len(A)):
            for j in range(len(A[0])):
                if (A[i,j]== 1):
                    r_ij_rel_g[i,j] = r_a[i] - r_a[j]
                    r_ij_rel_i[i,j] = C_ia[i] @ r_ij_rel_g[i,j]
        return r_ij_rel_i

    def error_norms(A,r_ij_rel_i, r_ij_rel_des_i):
        # 2*error norms. Errors have no directions
        r_ij_star = {}             # ||rij||**2 - ||rij_des||**2
        for i in range(len(A)):
            for j in range(len(A[0])):
                if(A[i,j]== 1 and j>i):
                    r_ij_star[i,j] = np.linalg.norm(r_ij_rel_i[i,j])**2 - np.linalg.norm(r_ij_rel_des_i[i,j])**2
        return r_ij_star

    def norms_for_plotting(A,r_ij_rel_i,r_ij_rel_des_i,u_iw_i, omega_ia_i):
        u = np.array(u_iw_i).reshape(-1,3)
        u_norm = np.linalg.norm(u)
        omega_ia_i_norm = np.linalg.norm(np.array(omega_ia_i))
        r_ij_star = Utilities.error_norms(A,r_ij_rel_i, r_ij_rel_des_i)
        e = 1/2*np.array(list(r_ij_star.values())).reshape(-1,1)
        return e, u_norm, omega_ia_i_norm