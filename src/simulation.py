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
import scipy.signal as sig
from model import A, B
import matplotlib.pyplot as plt

# Simulation Parameters
x0 = np.zeros(14)

start = 0
stop = 10
steps = 100
time = np.linspace(start,stop,steps)

# Input Definitions
U = np.zeros((len(time), 8))

h = 0.01       # step height (m)
tr = 0.01      # ramp time (s)

U[:,0] = h * np.clip(time / tr, 0, 1)        # Front Left displacent
U[:,4] = (h / tr) * (time < tr)              # Front Left velocity
U[:,1] = h * np.clip(time / tr, 0, 1)        # Front Right displacment
U[:,5] = (h / tr) * (time < tr)              # Front Right velocity


# Output Definitions
C = np.eye(14)
D = np.zeros((14,8))

# State Space
sys = sig.StateSpace(A,B,C,D)

t, y, x = sig.lsim(sys, U, time)

# Plotting
plt.figure(figsize=(12,9))
plt.plot(t,y[:, 0], color='blue',linewidth = 3)
plt.plot(t,y[:, 1], color='green',linewidth = 3)
plt.plot(t,y[:, 2], color='orange',linewidth = 3)
plt.title("Step Response")
plt.xlabel("t")
plt.ylabel("y")
plt.legend()
plt.grid()
plt.show()
