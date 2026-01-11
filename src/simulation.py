""" Railcar Suspension Simulation

Usage:
    Simulate railcar suspension dynamics over time. 
    Return carbody bounce, pitch, and roll, and bogie bounce and roll responses.

Author: Benjamin Sauer

Date:
    January 7th, 2026

Status: IN PROGRESS

"""

import numpy as np
from scipy.integrate import solve_ivp
from model import M_inv, C, K

