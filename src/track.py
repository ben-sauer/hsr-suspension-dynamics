""" Track-Disturbance Model

Usage:
    Generate input forces/displacements from the track

Author: Benjamin Sauer

Date:
    January 7th, 2026

Status: IN PROGRESS

"""

import numpy as np

# Step Track Input
def step_input_both(t, t0, amplitude):
    """
    Generate a step input at time t0 with given amplitude. Applied to both left and right rails.
    
    Parameters:
        t (float): Current time [s]
        t0 (float): Time at which the step occurs [s]
        amplitude (float): Amplitude of the step [m]
        
    Returns:
        list: Displacement from left and right rails at time t [m]
    """
    if t >= t0:
        return [amplitude, amplitude]
    else:
        return [0.0, 0.0]
    
# Sine Wave Track Input
