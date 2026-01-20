""" Railcar Suspension Simulation

Usage:
    Simulate railcar suspension dynamics over time. 
    Return carbody bounce, pitch, and roll, and bogie bounce and roll responses.

Author: Benjamin Sauer

Date:
    January 11th, 2026

Status: IN PROGRESS

"""

from scipy.signal import welch
import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
from model import A, B, b_1, M, C, K
from track import step_input, sin_input, rough_input

# Simulation Parameters
x0 = np.zeros(14)

start = 0 # [s]
stop = 15 # [s]
steps = 600
time = np.linspace(start,stop,steps)

# Step Input Parameters
h_L = 0.1       # Left displacement height [m]
h_R = 0.1       # Right displacement height [m]
tr_L = 0.01     # Left ramp time [s]
tr_R = 0.01     # Right ramp time [s]

### Uncomment below to use step response
# U = step_input(time, h_L, h_R, tr_L, tr_R)


# Sinusoidal Input Parameters
amp_L, amp_R = 0.03, 0.03     # Left and Rigth amplitudes [m]
freq_L, freq_R = 0.75, 0.75     # Left and Right frequencies [Hz]


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


# Solve for Acceleration
q = x[:, 0:7]
q_dot = x[:, 7:14]

x_dot = x @ A.T + U @ B.T
q_ddot = x_dot[:, 7:14]
a_z = q_ddot[:, 0]  

# Get PSD and Plot
fs = 1 / (t[1] - t[0])

f, Paa = welch(a_z, fs=fs, nperseg=4096, detrend='constant')

plt.figure(figsize=(12,9))
plt.semilogy(f, Paa)
plt.xlim(0, 10)     # rail vehicle comfort is low-frequency
plt.xlabel("Frequency [Hz]")
plt.ylabel("PSD of carbody accel [m^2/s^4/Hz]")
plt.title("Power-Spectral Density of Carbody Vertical Acceleration from Rough Track Input")
plt.minorticks_on()
plt.grid()
plt.savefig('figures/psd_az_3.png')
plt.show()


# # General Plotting
# plt.figure(figsize=(12,9))
# plt.plot(t,y[:, 0], color='blue',linewidth = 3, label='Carbody Bounce')
# # plt.plot(t,y[:, 1], color='green',linewidth = 3, label='Carbody Roll')
# # plt.plot(t,y[:, 2], color='orange',linewidth = 3, label='Carbody Pitch')
# # plt.plot(t,((y[:, 0] + b_1 * y[:, 2]) - y[:, 3]), color='red', linewidth = 3, label = 'Front Suspension Travel')
# plt.plot(t,q_ddot[:, 0], color='yellow',linewidth = 3, label='Carbody Bounce Acceleration')
# plt.title("Railcar Response")
# plt.xlabel("Time [s]")
# plt.ylabel("Y - Meters or Radians")
# plt.legend()
# plt.grid()
# plt.show()
