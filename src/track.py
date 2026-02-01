""" Track-Disturbance Model

Usage:
    Generate input forces/displacements from the track

Author: Benjamin Sauer

Date:
    January 7th, 2026

Status: FINISHED

"""

import numpy as np
import random
from model import b, v

# Random Seed
seed = 98765
random.seed(seed)

# Step Track Input
def step_input(t, h_L, h_R, tr_L, tr_R, t0_L = 0, t0_R = 0, v = v):
    """
    Description:
        Generate a step input for left and or right rails.
    
    Parameters:
        t (ndarray): Simulation time frame
        h_L (float): left step height [m]
        h_R (float): right step height [m]
        tr_L (float): left ramp time [s]
        tr_R (float): right ramp time [s]
        t0_L (float): left start time [s]
        t0_R (float): right start time [s]
        v (float): Current velocity of train [s]
        
    Returns:
        U matrix of inputs
    """
    # Input Definitions
    U = np.zeros((len(t), 8))

    # Check for Division by Zero
    if (tr_L == 0 or tr_R == 0):
        print("Please insert a non-zero value for tr_L and/or tr_R")
        return
    # Apply Step input
    else:
        U[:,0] = h_L * np.clip((t - t0_L) / tr_L, 0, 1)                  # Front Left displacent
        U[:,4] = (h_L / tr_L) * (((t - t0_L) >= 0) & ((t - t0_L) < tr_L))  # Front Left velocity
        U[:,1] = h_R * np.clip((t - t0_R) / tr_R, 0, 1)                  # Front Right displacment
        U[:,5] = (h_R / tr_R) * (((t - t0_R) >= 0) & ((t - t0_R) < tr_R))  # Front Right velocity

        # Set rear bogie inputs with delay
        t_delay = b / v
        U[:,2] = h_L * np.clip(((t - t0_L) - t_delay) / tr_L, 0, 1)                         # Rear Left displacment
        U[:,6] = (h_L / tr_L) * (((t - t0_L) >= t_delay) & ((t - t0_L) < t_delay + tr_L))     # Rear Left velocity
        U[:,3] = h_R * np.clip(((t - t0_R) - t_delay) / tr_R, 0, 1)                         # Rear Right displacment
        U[:,7] = (h_R / tr_R) * (((t - t0_R) >= t_delay) & ((t - t0_R) < t_delay + tr_R))     # Rear Right velocity

        return U
    

# Sine Wave Track Input
def sin_input(t, amp_L, amp_R, freq_L, freq_R, t0_L = 0, t0_R = 0, v = v):
    """
    Description:
        Generate sinusoidal input disturbance for left and/or right tracks

    Parameters:
        t (ndarray): Simulation time frame
        amp_L (float): Amplitude of left rail disturbance [m]
        amp_R (float): Amplitude of right trail disturbance [m]
        freq_L (float): Frequency of left rail disturbance [Hz]
        freq_R (float): Frequency of right rail disturbance [Hz]
        t0_L (float): left start time [s]
        t0_R (float): right start time [s]
        v (float): Current velocity of train [s]

    Returns:
        U matrix of inputs
    """
    # Input Definitions
    U = np.zeros((len(t), 8))

    # Apply Sinusoidal input
    U[:,0] = amp_L * np.sin(2 * np.pi * freq_L * (t - t0_L))     # Front Left displacent
    U[:,4] = amp_L * 2 * np.pi * freq_L * np.cos(2 * np.pi * freq_L * (t - t0_L))     # Front Left velocity
    U[:,1] = amp_R * np.sin(2 * np.pi * freq_R * (t - t0_R))     # Front Right displacment
    U[:,5] = amp_R * 2 * np.pi * freq_R * np.cos(2 * np.pi * freq_R * (t - t0_R))     # Front Right velocity

    # Set rear bogie inputs with delay
    t_delay = b / v
    U[:,2] = amp_L * np.sin(2 * np.pi * freq_L * (t - t0_L - t_delay))     # Rear Left displacment
    U[:,6] = amp_L * 2 * np.pi * freq_L * np.cos(2 * np.pi * freq_L * (t - t0_L - t_delay))     # Rear Left velocity
    U[:,3] = amp_R * np.sin(2 * np.pi * freq_R * (t - t0_R - t_delay))     # Rear Right displacment
    U[:,7] = amp_R * 2 * np.pi * freq_R * np.cos(2 * np.pi * freq_R * (t - t0_R - t_delay))     # Rear Right velocity

    return U


# PSD-based Roughness Track Model
def rough_input(t, start, stop, v = v, A = 20.95, p = 0.8, seed = None):
    """
    Description:
        Generate roughness profile for left and right rails. Meant to resemble physical rail.
        PSD and randomness model is based on M. Podwórna's "Modeling of Random Vertical Irregularities of Railway Tracks".
        See references document on GitHub for more information.
    
    Parameters:
        t (ndarray): Simulation time frame
        start, stop (float)): Start/stop time of simulation [s]
        v (float): Current velocity of train [s]
        A (float): Vertical track irregularity parameter [mm^2*rad/m] See simulation.py for detailed information on standards.
        p (float): Cross-level roughness. 1 = left and right rails identical. Decrease p to make left and right rail increasingly different.
        seed (int): RNG seed (for reproducability)
        
    Returns:
        U matrix of inputs
    """

    # Define Random Number Generator
    rng = np.random.default_rng(seed)

    # Define track space
    timeframe = float(stop - start)
    L = v * timeframe

    # Define wavelengths and frequency indices
    lambda_min = 0.10 # [m]
    lambda_max = 70.0 # [m]

    omega_min = 2*np.pi / lambda_max
    omega_max = 2*np.pi / lambda_min

    # Define dx and x range
    dt = np.diff(t)
    dt_min = float(np.min(dt))
    dx_from_time = v * dt_min
    dx_target = lambda_min / 10.0
    dx = min(lambda_min/2.0, max(dx_target, dx_from_time))

    # Define x range and FFT Resolution
    x = np.arange(-b, L + dx, dx)
    N = 1 << (len(x) - 1).bit_length()

    # FFT grid setup
    f = np.fft.rfftfreq(N, d=dx)     # [m^-1]
    omega = 2*np.pi*f                # [rad/m]
    df = 1.0/(N*dx)



    # PSD Function
    omega_c = 0.8242 # critical number [rad/m]
    
    def psd_function(omega):
        omega = np.asarray(omega)
        A_meters = A * 1e-6 # convert from mm^2 to m^2
        S_omega = (A_meters * omega_c**2) / ((omega**2 + omega_c**2)* (omega**2))
        return S_omega
    
    band = (omega >= omega_min) & (omega <= omega_max)

    S_f = 2*np.pi * psd_function(omega) # Convert radians to frequency
    S_f = np.where(band, S_f, 0.0)


    # Inverse Fast Fourier Transform
    def make_track():
        X = np.zeros(len(f), dtype=np.complex128)

        # random complex normals
        a = rng.standard_normal(len(f))
        b_im = rng.standard_normal(len(f))

        scale = 0.5 * N * np.sqrt(S_f * df)  # per-bin scaling
        X = (a + 1j * b_im) * scale

        X[0] = 0.0 + 0.0j
        if N % 2 == 0:
            X[-1] = 0.0 + 0.0j

        r = np.fft.irfft(X, n=N) 
        return r[:len(x)]        

    r_0 = make_track()
    r_1 = make_track()

    # Left rail = r0; Right rail correlated with coefficient p
    p = float(np.clip(p, 0.0, 1.0))
    r_L = r_0
    r_R = p * r_0 + np.sqrt(1.0 - p**2) * r_1



    # Translate to time domain for inputs
    x_int = v * t
    
    
    U = np.zeros((len(t), 8))

    U[:,0] = np.interp(x_int, x, r_L)   # Front Left displacent
    U[:,4] = np.gradient(U[:,0], t)               # Front Left velocity
    U[:,1] = np.interp(x_int, x, r_R)   # Front Right displacent
    U[:,5] = np.gradient(U[:,1], t)               # Front Right velocity

    U[:,2] = np.interp(x_int - b, x, r_L)   # Rear Left displacent
    U[:,6] = np.gradient(U[:,2], t)               # Rear Left velocity
    U[:,3] = np.interp(x_int - b, x, r_R)   # Rear Right displacent
    U[:,7] = np.gradient(U[:,3], t)               # Rear Right velocity

    return U