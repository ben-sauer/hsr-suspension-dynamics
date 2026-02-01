""" Supspension Controller Module

Usage:
    Control secondary suspension damping based on carbody motion. Semi-active control.

Author: Benjamin Sauer

Date:
    January 7th, 2026

Status: IN PROGRESS

"""

import numpy as np
from model import c_sL1, b_1, b_2, l_L, l_R, assemble_C_matrix

default_c = c_sL1

# Passive Damping Controller
def passive_damping_controller():
    """
    Description:
        Passive damping controller for secondary suspension.
    
    Returns:
        dict: Damping coefficients for left and right secondary suspensions
    """
    # Fixed damping coefficients (Ns/m)
    c_sL1 = 2.0e4
    c_sR1 = 2.0e4
    c_sL2 = 2.0e4
    c_sR2 = 2.0e4

    return {
        'c_sL1': c_sL1,
        'c_sR1': c_sR1,
        'c_sL2': c_sL2,
        'c_sR2': c_sR2
    }

# Skyhook Damping Controller
def skyhook_damping_controller(q_dot):
    """
    Description:
        Semi-active damping controller for secondary suspension.

    Parameters:
        q_dot = Velocities of each of the seven degrees of freedom - Index 7-14 in state vector

    Returns:
        dict: Damping coefficients for left and right secondary suspensions
    """

    # Max and Min Damping Coefficients
    c_max = default_c * 5
    c_min = default_c * 0.1

    # Decompose q_dot
    zd_c = q_dot[0]
    phid_c = q_dot[1]
    thetad_c = q_dot[2]
    zd_b1 = q_dot[3]
    phid_b1 = q_dot[4]
    zd_b2 = q_dot[5]
    phid_b2 = q_dot[6]


    # Front Right Secondary Damper
    if ((zd_c + b_1*thetad_c + l_R*phid_c)*((zd_c + b_1*thetad_c + l_R*phid_c) - (zd_b1 + l_R*phid_b1))) >= 0:
        c_sL1 = c_max
    else:
        c_sL1 = c_min

    # Front Left Secondary Damper
    if ((zd_c + b_1*thetad_c - l_L*phid_c)*((zd_c + b_1*thetad_c - l_L*phid_c) - (zd_b1 - l_L*phid_b1))) >= 0:
        c_sR1 = c_max
    else:
        c_sR1 = c_min

    # Rear Right Secondary Damper
    if ((zd_c - b_2*thetad_c + l_R*phid_c)*((zd_c - b_2*thetad_c + l_R*phid_c) - (zd_b2 + l_R*phid_b2))) >= 0:
        c_sL2 = c_max
    else:
        c_sL2 = c_min

    # Rear Left Secondary Damper
    if ((zd_c - b_2*thetad_c - l_L*phid_c)*((zd_c - b_2*thetad_c - l_L*phid_c) - (zd_b2 - l_L*phid_b2))) >= 0:
        c_sR2 = c_max
    else:
        c_sR2 = c_min

    C_sky = assemble_C_matrix(c_sL1=c_sL1, c_sR1=c_sR1, c_sL2=c_sL2, c_sR2=c_sR2)
    return C_sky