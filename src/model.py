""" Railcar-Model Assembly

Usage:
    Assemble railcar matrices and components.

Author:
    Benjamin Sauer

Date:
    January 7, 2026

Status: FINISHED

"""

import numpy as np


# ==========================
#         Parameters
# ==========================

v = 310*3600/1000 # Railcar velocity [m/s]
# v = 800*3600/1000

m_c = 47800 # Carbody mass [kg]
m_b1, m_b2 = 3500, 3500 # Bogie masses [kg]

I_cx = 119328 # Carbody roll moment of inertia [kg*m^2]
I_cy = 1957888 # Carbody pitch moment of inertia [kg*m^2]
I_b1, I_b2 = 2835, 2835 # Bogie roll moment of inertia [kg*m^2]

b_1, b_2 = 17.375//2, 17.375//2 # Distances from carbody CG to bogie CGs [m]
b = 17.375 # Distance between Front and Rear Bogie CG
l_L, l_R = 2//2, 2//2 # Distances from bogie CG to left/right wheelsets [m]

k_pL1, k_pR1, k_pL2, k_pR2 = 1.2e6, 1.2e6, 1.2e6, 1.2e6 # Primary suspension stiffnesses [N/m]
c_pL1, c_pR1, c_pL2, c_pR2 = 1.0e4, 1.0e4, 1.0e4, 1.0e4 # Primary suspension damping coefficients [Ns/m]
k_sL1, k_sR1, k_sL2, k_sR2 = 3.5e5, 3.5e5, 3.5e5, 3.5e5 # Secondary suspension stiffnesses [N/m]
c_sL1, c_sR1, c_sL2, c_sR2 = 2.0e4, 2.0e4, 2.0e4, 2.0e4 # Secondary suspension damping coefficients [Ns/m]

# Inputs placeholders
z_gL1, z_gR1, z_gL2, z_gR2 = 0, 0, 0, 0 # Track irregularities at wheel-rail contacts [m]

# Define Coordinates
# z_c, phi_c, theta_c: Carbody vertical, roll, pitch
# z_b1, phi_b1: Front bogie vertical, roll
# z_b2, phi_b2: Rear bogie vertical, roll
# q = [z_c, phi_c, theta_c, z_b1, phi_b1, z_b2, phi_b2]



# ==============================
#       Matrix Construction
# ==============================

M = np.array([[m_c, 0, 0, 0, 0, 0, 0],
              [0, I_cx, 0, 0, 0, 0, 0],
              [0, 0, I_cy, 0, 0, 0, 0],
              [0, 0, 0, m_b1, 0, 0, 0],
              [0, 0, 0, 0, I_b1, 0, 0],
              [0, 0, 0, 0, 0, m_b2, 0],
              [0, 0, 0, 0, 0, 0, I_b2]])

M_inv = np.linalg.inv(M)

def assemble_C_matrix(c_sL1=c_sL1, c_sR1=c_sR1, c_sL2=c_sL2, c_sR2=c_sR2):
    C = np.array([[c_sL1 + c_sR1 + c_sL2 + c_sR2, -l_L*(c_sL1 + c_sL2) + l_R*(c_sR1 + c_sR2), b_1*(c_sL1 + c_sR1) - b_2*(c_sL2 + c_sR2), -(c_sL1 + c_sR1), c_sL1*l_L - c_sR1*l_R, -(c_sL2 + c_sR2), c_sL2*l_L - c_sR2*l_R],
                [-l_L*(c_sL1 + c_sL2) + l_R*(c_sR1 + c_sR2), l_L**2*(c_sL1 + c_sL2) + l_R**2*(c_sR1 + c_sR2), b_1*(-l_L*c_sL1 + l_R*c_sR1) - b_2*(-l_L*c_sL2 + l_R*c_sR2), l_L*c_sL1 - l_R*c_sR1, -l_L**2*c_sL1 - l_R**2*c_sR1, l_L*c_sL2 - l_R*c_sR2, -l_L**2*c_sL2 - l_R**2*c_sR2],
                [b_1*(c_sL1 + c_sR1) - b_2*(c_sL2 + c_sR2), b_1*(-l_L*c_sL1 + l_R*c_sR1) - b_2*(-l_L*c_sL2 + l_R*c_sR2), b_1**2*(c_sL1 + c_sR1) + b_2**2*(c_sL2 + c_sR2), -b_1*(c_sL1 + c_sR1), b_1*(l_L*c_sL1 - l_R*c_sR1), b_2*(c_sL2 + c_sR2), b_2*(l_L*c_sL2 - l_R*c_sR2)],
                [-(c_sL1 + c_sR1), l_L*c_sL1 - c_sR1*l_R, -b_1*(c_sL1 + c_sR1), (c_sL1 + c_sR1 + c_pL1 + c_pR1), -l_L*(c_sL1+c_pL1) + l_R*(c_sR1 + c_pR1), 0, 0],
                [c_sL1*l_L - c_sR1*l_R, -l_L**2*c_sL1 - l_R**2*c_sR1, b_1*(l_L*c_sL1 - l_R*c_sR1), -l_L*(c_sL1+c_pL1) + l_R*(c_sR1 + c_pR1), l_L**2*(c_sL1 + c_pL1) + l_R**2*(c_sR1 + c_pR1), 0, 0],
                [-(c_sL2 + c_sR2), l_L*c_sL2 - c_sR2*l_R, b_2*(c_sL2 + c_sR2), 0, 0, (c_sL2 + c_sR2 + c_pL2 + c_pR2), -l_L*(c_sL2 + c_pL2) + l_R*(c_sR2 + c_pR2)],
                [c_sL2*l_L - c_sR2*l_R, -l_L**2*c_sL2 - l_R**2*c_sR2, b_2*(l_L*c_sL2 - l_R*c_sR2), 0, 0, -l_L*(c_sL2 + c_pL2) + l_R*(c_sR2 + c_pR2), l_L**2*(c_sL2 + c_pL2) + l_R**2*(c_sR2 + c_pR2)]])
    return C

C = assemble_C_matrix()

# Create K matrix similarly, changing c's to k's
K = np.array([[k_sL1 + k_sR1 + k_sL2 + k_sR2, -l_L*(k_sL1 + k_sL2) + l_R*(k_sR1 + k_sR2), b_1*(k_sL1 + k_sR1) - b_2*(k_sL2 + k_sR2), -(k_sL1 + k_sR1), k_sL1*l_L - k_sR1*l_R, -(k_sL2 + k_sR2), k_sL2*l_L - k_sR2*l_R],
              [-l_L*(k_sL1 + k_sL2) + l_R*(k_sR1 + k_sR2), l_L**2*(k_sL1 + k_sL2) + l_R**2*(k_sR1 + k_sR2), b_1*(-l_L*k_sL1 + l_R*k_sR1) - b_2*(-l_L*k_sL2 + l_R*k_sR2), l_L*k_sL1 - l_R*k_sR1, -l_L**2*k_sL1 - l_R**2*k_sR1, l_L*k_sL2 - l_R*k_sR2, -l_L**2*k_sL2 - l_R**2*k_sR2],
              [b_1*(k_sL1 + k_sR1) - b_2*(k_sL2 + k_sR2), b_1*(-l_L*k_sL1 + l_R*k_sR1) - b_2*(-l_L*k_sL2 + l_R*k_sR2), b_1**2*(k_sL1 + k_sR1) + b_2**2*(k_sL2 + k_sR2), -b_1*(k_sL1 + k_sR1), b_1*(l_L*k_sL1 - l_R*k_sR1), b_2*(k_sL2 + k_sR2), b_2*(l_L*k_sL2 - l_R*k_sR2)],
              [-(k_sL1 + k_sR1), l_L*k_sL1 - k_sR1*l_R, -b_1*(k_sL1 + k_sR1), (k_sL1 + k_sR1 + k_pL1 + k_pR1), -l_L*(k_sL1+k_pL1) + l_R*(k_sR1 + k_pR1), 0, 0],
              [k_sL1*l_L - k_sR1*l_R, -l_L**2*k_sL1 - l_R**2*k_sR1, b_1*(l_L*k_sL1 - l_R*k_sR1), -l_L*(k_sL1+k_pL1) + l_R*(k_sR1 + k_pR1), l_L**2*(k_sL1 + k_pL1) + l_R**2*(k_sR1 + k_pR1), 0, 0],
              [-(k_sL2 + k_sR2), l_L*k_sL2 - k_sR2*l_R, b_2*(k_sL2 + k_sR2), 0, 0, (k_sL2 + k_sR2 + k_pL2 + k_pR2), -l_L*(k_sL2 + k_pL2) + l_R*(k_sR2 + k_pR2)],
              [k_sL2*l_L - k_sR2*l_R, -l_L**2*k_sL2 - l_R**2*k_sR2, b_2*(l_L*k_sL2 - l_R*k_sR2), 0, 0, -l_L*(k_sL2 + k_pL2) + l_R*(k_sR2 + k_pR2), l_L**2*(k_sL2 + k_pL2) + l_R**2*(k_sR2 + k_pR2)]])

# Create input G matrices
G_z_dot = np.array([[0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [c_pL1,c_pR1,0,0],
              [-l_L*c_pL1,l_R*c_pR1,0,0],
              [0,0,c_pL2,c_pR2],
              [0,0,-l_L*c_pL2,l_R*c_pR2]])

G_z = np.array([[0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [k_pL1,k_pR1,0,0],
              [-l_L*k_pL1,l_R*k_pR1,0,0],
              [0,0,k_pL2,k_pR2],
              [0,0,-l_L*k_pL2,l_R*k_pR2]])

def assemble_A_matrix(M_inv=M_inv, C=C, K=K):
    """ Assemble A matrix for state-space representation.

    Args:
        M_inv (np.ndarray): Inverse mass matrix.
        C (np.ndarray): Damping matrix.
        K (np.ndarray): Stiffness matrix.
    Returns:
        A (np.ndarray): State matrix
        """
    Z = np.zeros_like(M_inv)
    I = np.eye(M_inv.shape[0])

    A_top = np.hstack((Z, I))
    A_bottom = np.hstack((-M_inv @ K, -M_inv @ C))
    A = np.vstack((A_top, A_bottom))

    return A

A = assemble_A_matrix()

def assemble_B_matrix(M_inv=M_inv,G_z=G_z,G_z_dot=G_z_dot):
    """ Assemble B matrix for state-space representation.

    Args:
        M_inv (np.ndarray): Inverse mass matrix.
        u (np.ndarray): Input vector.
        G_z: Input position displacement matrix
        G_z_dot: Input velocity displacement matrix
    Returns:
        B (np.ndarray): Input matrix
    """
    G = np.hstack((G_z, G_z_dot))
    Z = np.zeros((7,8))
    B_bottom = M_inv @ G
    B = np.vstack((Z, B_bottom))

    return B

B = assemble_B_matrix()

