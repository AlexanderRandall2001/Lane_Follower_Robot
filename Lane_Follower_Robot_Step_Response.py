import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, step

# PD-controlled lateral system step response simulation.

# This script:
# - Defines a second-order system
# - Simulates step response
# - Plots lateral position over time

# Parameters

v = 0.1       # forward speed (m/s)

# PD gains

Kp = 40
Kd = 40

# Transfer function: Y(s)/Yref(s) = v(Kd*s + Kp)/(s^2 + v*Kd*s + v*Kp)

num = [ v * Kp]
den = [1, v * Kd, v * Kp]

system = TransferFunction(num, den)

# Step response

t = np.linspace(0, 10, 1000)  # simulate 10 seconds
t, y = step(system, T=t)

# Plot

plt.figure(figsize=(8,5))
plt.plot(t, y, 'g', linewidth=2, label='Lateral Position Response')
plt.xlabel('Time (s)')
plt.ylabel('Lateral Position y (m)')
plt.title('Step Response of PD-Controlled System')
plt.grid(True)
plt.legend()
plt.show()