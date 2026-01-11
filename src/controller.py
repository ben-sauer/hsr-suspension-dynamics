""" Supspension Controller Module

Usage:
    Control secondary suspension damping based on carbody motion. Semi-active control.

Author: Benjamin Sauer

Date:
    January 7th, 2026

Status: IN PROGRESS

"""

# Passive Damping Controller
def passive_damping_controller():
    """
    Passive damping controller for secondary suspension.
    
    Returns:
        dict: Damping coefficients for left and right secondary suspensions
    """
    # Fixed damping coefficients (Ns/m)
    c_sL1 = 1.0e4
    c_sR1 = 1.0e4
    c_sL2 = 1.0e4
    c_sR2 = 1.0e4
    
    return {
        'c_sL1': c_sL1,
        'c_sR1': c_sR1,
        'c_sL2': c_sL2,
        'c_sR2': c_sR2
    }