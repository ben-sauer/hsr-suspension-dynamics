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
from model import A, B, b
import matplotlib.pyplot as plt

# Simulation Parameters
x0 = np.zeros(14)

start = 0
stop = 10
steps = 200
time = np.linspace(start,stop,steps)

# Input Definitions
U = np.zeros((len(time), 8))

v = 310*3600/1000   # velocity (m/s)
h = 0.01            # step height (m)
tr = 0.01           # ramp time (s)

U[:,0] = h * np.clip(time / tr, 0, 1)            # Front Left displacent
U[:,4] = (h / tr) * ((time >= 0) & (time < tr))  # Front Left velocity
U[:,1] = h * np.clip(time / tr, 0, 1)            # Front Right displacment
U[:,5] = (h / tr) * ((time >= 0) & (time < tr))  # Front Right velocity

# Set rear bogie inputs with delay
t_delay = b / v
U[:,2] = h * np.clip((time - t_delay) / tr, 0, 1)                   # Rear Left displacment
U[:,6] = (h / tr) * ((time >= t_delay) & (time < t_delay + tr))     # Rear Left velocity
U[:,3] = h * np.clip((time - t_delay) / tr, 0, 1)                   # Rear Right displacment
U[:,7] = (h / tr) * ((time >= t_delay) & (time < t_delay + tr))     # Rear Right velocity


# Output Definitions
C = np.eye(14)
D = np.zeros((14,8))

# State Space
sys = sig.StateSpace(A,B,C,D)

t, y, x = sig.lsim(sys, U, time)

# Plotting
plt.figure(figsize=(12,9))
plt.plot(t,y[:, 0], color='blue',linewidth = 3, label='Carbody Bounce')
plt.plot(t,y[:, 1], color='green',linewidth = 3, label='Carbody Roll')
plt.plot(t,y[:, 2], color='orange',linewidth = 3, label='Carbody Pitch')
plt.title("Step Response")
plt.xlabel("Time [s]")
plt.ylabel("Y - Meters or Radians")
plt.legend()
plt.grid()
plt.show()
