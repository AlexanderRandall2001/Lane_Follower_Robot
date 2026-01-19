import math
import matplotlib.pyplot as plt

# PD control simulation for sinusoidal path tracking   

# The robot tracks y_ref = A * sin(Bx)

# Includes:
# - PD lateral controller
# - Kinematic differential drive model
# - Path comparison plot

# Simulation parameters

dt = 0.01
T = 30.0  
steps = int(T / dt)

# Robot parameters (e-puck)

wheel_radius = 0.0205  
L = 0.052 # wheelbase length

v = 0.1  # forward speed

# PD Controller Parameters

Kp = 40
Kd = 40

# Initial conditions

x = 0.0
y = 0.0
theta = 0.0

prev_error = 0.0


# Sinusoidal reference path
# y_ref = A sin(Bx)

A = 0.1
B = math.pi

# Plot Data

xs = []
ys = []
y_refs = []

# Simulation loop

for _ in range(steps):

    # Reference
    y_ref = A * math.sin(B * x)
    y_refs.append(y_ref)

    # PD control
    error = y - y_ref
    derivative = (error - prev_error) / dt
    omega = -(Kp * error + Kd * derivative)
    prev_error = error

    # Differential drive (m/s)
    v_r = v + (L / 2) * omega
    v_l = v - (L / 2) * omega

    # True Position and angle
    v_actual = (v_r + v_l) / 2
    omega_actual = (v_r - v_l) / L

    theta += omega_actual * dt
    x += v_actual * math.cos(theta) * dt
    y += v_actual * math.sin(theta) * dt

    xs.append(x)
    ys.append(y)


# Plot

plt.figure(figsize=(12,6))
plt.plot(xs, y_refs, 'r', linewidth=2, label="Target Path")
plt.plot(xs, ys, 'g', linewidth=2, label="Actual Robot Path")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("PD Line Following Sinosoidal Path")
plt.legend()
plt.grid(True)
plt.show()