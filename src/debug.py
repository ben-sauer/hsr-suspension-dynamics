# Temporary Testing Simulation 2

import numpy as np
from model import M, M_inv, C, K, A, B
import math
from scipy.integrate import solve_ivp
import scipy.linalg
import matplotlib.pyplot as plt

# print(np.linalg.eigvals(C))
# print(np.linalg.eigvals(K))

# print(np.max(np.abs(C - C.T)))
# print(C)
# print(np.diag(C))

# print(M)

# print("K min eig:", np.min(np.linalg.eigvalsh((K+K.T)/2)))
# print(B)

def truncate(number, decimals):
    """Truncates a number to a specified number of decimal places."""
    if decimals < 0:
        raise ValueError("Decimals must be a non-negative integer")
    factor = 10.0**decimals
    return math.trunc(number * factor) / factor

eigvals, eigvecs = scipy.linalg.eig(K, M)
omega_n = np.sqrt(np.real(eigvals))
freq_hz = omega_n / (2*np.pi)

print(freq_hz)

# for i in range(len(eigvals)):
#     print(truncate(freq_hz[i], 3), eigvecs[i])
#     print('\n')

