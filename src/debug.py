# Temporary Testing Simulation 2

import numpy as np
from model import M, M_inv, C, K, A, B
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# print(np.linalg.eigvals(C))
# print(np.linalg.eigvals(K))

# print(np.max(np.abs(C - C.T)))
# print(C)
# print(np.diag(C))

# print(M)

# print("K min eig:", np.min(np.linalg.eigvalsh((K+K.T)/2)))
print(B)
