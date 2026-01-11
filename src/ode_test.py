# Temporary Testing Simulation

import numpy as np
from model import M_inv, C, K
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Quick ODE Constructor
def ode_system(t, x, f):

    # Define States
    x = np.array(x)

    q1 = np.array(x[0:7]) # Position
    q2 = np.array(x[7:14]) # Velocity
    # print(len(q1),len(q2))

    # Equation
    q2_dot = np.linalg.solve(M_inv, (f - C @ q2 - K @ q1))

    
    return np.concatenate([q2,q2_dot])

# Time
t_span = (0, 5)
t_eval = np.linspace(t_span[0], t_span[1], 30)

# Initial Values
f = np.array([10,0,0,0,0,0,0])
q1_0 = np.zeros(7)
q2_0 = np.zeros(7)
x0 = np.concatenate([q1_0,q2_0])

# ODE Solver
solution = solve_ivp(
    ode_system,
    (0, 10),
    x0,
    method="Radau",
    args=(f,),
    rtol=1e-4,
    atol=1e-7,
    max_step=1e-2,     # start here
    first_step=1e-4
)

# solution = solution.reshape(2,7)

#Plot Results
plt.figure(figsize=(10,6))
plt.plot(solution.t,solution.y[0],label='x(t)')
plt.plot(solution.t,solution.y[7],label='x_dot(t)')

plt.xlim(-1,11)
plt.ylim(-10, 10) 

plt.title("Solution")
plt.legend()
plt.grid()
plt.show()