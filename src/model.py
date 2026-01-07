""" Railcar-Model Assembly

Usage:
    Assemble railcar matrices and components.

Author:
    Benjamin Sauer

Date:
    January 7, 2026

Status: IN PROGRESS    

"""

import numpy as np
from scipy.sparse import csr_matrix, bmat

# Parameters
m_c = 47800 # Carbody mass [kg]
m_b1, m_b2 = 3500, 3500 # Bogie masses [kg]

I_cx = 1000 # Carbody roll moment of inertia [kg*m^2]
I_cy = 100 # Carbody pitch moment of inertia [kg*m^2]
I_b1, I_b2 = 2835, 2835 # Bogie roll moment of inertia [kg*m^2]

b_1, b_2 = 17.375//2, 17.375//2 # Distances from carbody CG to bogie CGs [m]
l_L, l_R = 2/2, 2/2 # Distances from bogie CG to left/right wheelsets [m]

k_pL1, k_pR1, k_pL2, k_pR2 = 1.2e6, 1.2e6, 1.2e6, 1.2e6 # Primary suspension stiffnesses [N/m]
c_pL1, c_pR1, c_pL2, c_pR2 = 1.0e4, 1.0e4, 1.0e4, 1.0e4 # Primary suspension damping coefficients [Ns/m]
k_sL1, k_sR1, k_sL2, k_sR2 = 3.5e5, 3.5e5, 3.5e5, 3.5e5 # Secondary suspension stiffnesses [N/m]
c_sL1, c_sR1, c_sL2, c_sR2 = 2.0e4, 2.0e4, 2.0e4, 2.0e4 # Secondary suspension damping coefficients [Ns/m]

M = np.array([[m_c, 0, 0, 0, 0, 0, 0],
              [0, I_cx, 0, 0, 0, 0, 0],
              [0, 0, I_cy, 0, 0, 0, 0],
              [0, 0, 0, m_b1, 0, 0, 0],
              [0, 0, 0, 0, I_b1, 0, 0],
              [0, 0, 0, 0, 0, m_b2, 0],
              [0, 0, 0, 0, 0, 0, I_b2]])

# Define Coordinates
# x_c, phi_c, theta_c: Carbody vertical, roll, pitch
# x_b1, phi_b1: Front bogie vertical, roll
# x_b2, phi_b2: Rear bogie vertical, roll
# q = [x_c, phi_c, theta_c, x_b1, phi_b1, x_b2, phi_b2]

C = np.array([[c_sL1 + c_sR1 + c_sL2 + c_sR2, -l_L*(c_sL1 + c_sL2) + l_R(c_sR1 + c_sR2), b_1*(c_sL1 + c_sR1) + b_2(c_sL2 + c_sR2), -(c_sL1 + c_sR1), c_sL1*l_L - c_sR1*l_R, -(c_sL2 + c_sR2), c_sL2*l_L - c_sR2*l_R],
              [-l_L*(c_sL1 + c_sL2) + l_R(c_sR1 + c_sR2), l_L**2*(c_sL1 + c_sL2) + l_R**2*(c_sR1 + c_sR2), b_1*(-l_L*c_sL1 + l_R*c_sR1) + b_2*(l_L*c_sL2 - l_R*c_sR2), l_L*c_sL1 - l_R*c_sR1, -l_L**2*c_sL1 - l_R**2*c_sR1, l_L*c_sL2 - l_R*c_sR2, -l_L**2*c_sL2 - l_R**2*c_sR2],
              [b_1*(c_sL1 + c_sR1) + b_2(c_sL2 + c_sR2), b_1*(-l_L*c_sL1 + l_R*c_sR1) + b_2*(l_L*c_sL2 - l_R*c_sR2), b_1**2*(c_sL1 + c_sR1) + b_2**2*(c_sL2 + c_sR2), b_1(c_sL1 - c_sR1), b_1*(l_L*c_sL1 - l_R*c_sR1), b_2*(c_sL2 + c_sR2), -b_2*(l_L*c_sL2 - l_R*c_sR2)],
              [-(c_sL1 + c_sR1), l_L*c_sL1 - c_sR1*l_R, -b_1*(c_sL1 + c_sR1), -(c_sL1 + c_sR1 + c_pL1 + c_pR1), l_L*(c_sL1+c_pL1) - l_R*(c_sR1 + c_pR1), 0, 0],
              [c_sL1*l_L - c_sR1*l_R, -l_L**2*c_sL1 - l_R**2*c_sR1, b_1*(l_L*c_sL1 - l_R*c_sR1), l_L*(c_sL1+c_pL1) - l_R*(c_sR1 + c_pR1), -l_L**2*(c_sL1 + c_pL1) - l_R**2*(c_sR1 + c_pR1), 0, 0],
              [-(c_sL2 + c_sR2), l_L*c_sL2 - c_sR2*l_R, b_2*(c_sL2 + c_sR2), 0, 0, -(c_sL2 + c_sR2 + c_pL2 + c_pR2), l_L*(c_sL2 + c_pL2) - l_R*(c_sR2 + c_pR2)],
              [c_sL2*l_L - c_sR2*l_R, -l_L**2*c_sL2 - l_R**2*c_sR2, b_2*(l_L*c_sL2 - l_R*c_sR2), 0, 0, l_L*(c_sL2 + c_pL2) - l_R*(c_sR2 + c_pR2), -l_L**2*(c_sL2 + c_pL2) - l_R**2*(c_sR2 + c_pR2)]])
              


class RailcarModel():
    #Assembly of railcar matrices and components
    def __init__(self, carbody, bogie_front, bogie_rear):
        self.carbody = carbody
        self.bogie_front = bogie_front
        self.bogie_rear = bogie_rear

        self.M = self.assemble_mass_matrix()
        self.C = self.assemble_damping_matrix()
        self.K = self.assemble_stiffness_matrix()


