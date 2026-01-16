""" Railcar Suspension Simulation

Usage:
    Simulate railcar suspension dynamics over time. 
    Return carbody bounce, pitch, and roll, and bogie bounce and roll responses.

Author: Benjamin Sauer

Date:
    January 11th, 2026

Status: IN PROGRESS

"""

import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
from model import A, B
from track import step_input, sin_input, rough_input

# Simulation Parameters
x0 = np.zeros(14)

start = 0
stop = 10
steps = 300
time = np.linspace(start,stop,steps)

# Step Input Parameters
h_L = 0.1       # Left displacement height
h_R = 0.1       # Right displacement height
tr_L = 0.01     # Left ramp time
tr_R = 0.01     # Right ramp time

### Uncomment below to use step response
# U += step_input(time, h_L, h_R, tr_L, tr_R)


# Sinusoidal Input Parameters
amp_L, amp_R = 0.01, 0.01     # Left and Rigth amplitudes
freq_L, freq_R = 6, 6


### Uncomment below to use sinusoidal response
# U = sin_input(time, amp_L, amp_R, freq_L, freq_R)


# Rough Input Parameters

### Uncomment below to use rough track response
U = rough_input(time, start, stop)

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
plt.title("Railcar Response")
plt.xlabel("Time [s]")
plt.ylabel("Y - Meters or Radians")
plt.legend()
plt.grid()
plt.show()
