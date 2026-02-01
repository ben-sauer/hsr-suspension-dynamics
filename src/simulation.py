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
from model import A, B, b_1, M, C, K, l_R, assemble_A_matrix
from track import step_input, sin_input, rough_input
from controller import skyhook_damping_controller


# =====================================
#         Simulation Parameters
# =====================================

x0 = np.zeros(14)

start = 0 # [s]
stop = 15 # [s]
steps = 1000
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


# # =====================================
# #             State Space
# # =====================================

# # Output Definitions
# C = np.eye(14)
# D = np.zeros((14,8))

# sys = sig.StateSpace(A,B,C,D)
# t, y, x = sig.lsim(sys, U, time)


# =====================================
#             ODE-Solver
# =====================================

def rk4_step(f, t, x, dt): # Source - ChatGPT
    k1 = f(t, x)
    k2 = f(t + 0.5*dt, x + 0.5*dt*k1)
    k3 = f(t + 0.5*dt, x + 0.5*dt*k2)
    k4 = f(t + dt, x + dt*k3)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def make_u_t(time, U):
    # simple linear interpolation for each input channel
    def u_t(t):
        return np.array([np.interp(t, time, U[:, j]) for j in range(U.shape[1])])
    return u_t

# Skyhook Simulator
def simulate_skyhook(U, time=time, x0=x0):
    u_t = make_u_t(time, U)

    X = np.zeros((len(time), 14))
    X[0] = x0

    def xdot(t, x):
        q_dot = x[7:]

        C_sky = skyhook_damping_controller(q_dot)
        A = assemble_A_matrix(C = C_sky)

        x_dot = A @ x + B @ u_t(t)

        return x_dot

    for i in range(len(time) - 1):
        dt = time[i + 1] - time[i]
        X[i+1] = rk4_step(xdot, time[i], X[i], dt)

    return X
    
x = simulate_skyhook(U)



# ===============================
#        RMS-Acceleration
# ===============================

# Solve for Vertical Acceleration
q = x[:, 0:7]
q_dot = x[:, 7:14]

x_dot = x @ A.T + U @ B.T
q_ddot = x_dot[:, 7:14]
a_z = q_ddot[:, 0]  

# Solve for Front Right Suspension Travel
ds = (x[:, 0] + l_R * x[:, 1] + b_1 * x[:, 2]) - x[:, 3]


# Solve for RMS Vertical Acceleration
t0 = int(0.3 * steps)
az_ss = a_z[t0:] # Start calculations after vibrations achieved steady state
ds_ss = ds[t0:]

az_rms = np.sqrt((1 / len(az_ss)) * np.mean(az_ss**2))
ds_rms = np.sqrt((1 / len(ds_ss)) * np.mean(ds_ss**2))


# Print RMS Acceleration and Suspension Travel
print(f"RMS Carbody Vertical Acceleration: {az_rms:.4f} m/s²")
print(f"RMS Suspension Travel: {ds_rms:.4f} m")



# # ====================================
# #            Get PSD Plot
# # ====================================

# fs = 1 / (t[1] - t[0])

# f, Paa = welch(a_z, fs=fs, nperseg=4096, detrend='constant')

# plt.figure(figsize=(12,9))
# plt.semilogy(f, Paa)
# plt.xlim(0, 10)     # rail vehicle comfort is low-frequency
# plt.xlabel("Frequency [Hz]")
# plt.ylabel("PSD of carbody accel [m^2/s^4/Hz]")
# plt.title("Power-Spectral Density of Carbody Vertical Acceleration from Rough Track Input")
# plt.minorticks_on()
# plt.grid()
# plt.savefig('figures/psd_az_3.png')
# plt.show()



# =====================================
#           Plotting Solutions
# =====================================

plt.figure(figsize=(12,9))
# plt.plot(t,x[:, 0], color='blue',linewidth = 3, label='Carbody Bounce')
# plt.plot(t,x[:, 1], color='green',linewidth = 3, label='Carbody Roll')
# plt.plot(t,x[:, 2], color='orange',linewidth = 3, label='Carbody Pitch')
# plt.plot(t,((x[:, 0] + b_1 * x[:, 2]) - x[:, 3]), color='red', linewidth = 3, label = 'Front Suspension Travel')
# plt.plot(t,q_ddot[:, 0], color='yellow',linewidth = 3, label='Carbody Bounce Acceleration')
plt.plot(time,x[:,0], color='purple', linewidth = 3, label = 'Carbody Bounce Acceleration - Skyhook')
plt.title("Railcar Response")
plt.xlabel("Time [s]")
plt.ylabel("Y - Meters or Radians")
plt.legend()
plt.grid()
plt.show()
