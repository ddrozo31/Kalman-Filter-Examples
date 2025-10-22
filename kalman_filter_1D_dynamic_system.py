#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Simulate true position with constant velocity
num_steps = 30
dt = 1.0
true_velocity = 1.0
true_positions = np.array([i * true_velocity for i in range(num_steps)])

# Noisy measurements of position
measurement_noise_std = 2.0
measurements = true_positions + np.random.normal(0, measurement_noise_std, num_steps)

# Kalman Filter (single-state: position only)
x_est = 0.0  # Initial position estimate
P = 1000.0   # Initial uncertainty

# Assume constant known velocity
assumed_velocity = 1.0

# Define process and measurement noise
Q = 1.0  # Process noise variance
R = measurement_noise_std ** 2  # Measurement noise variance

# Store estimates
estimates = []

for z in measurements:
    # Prediction (use assumed velocity)
    x_pred = x_est + assumed_velocity * dt
    P_pred = P + Q

    # Update
    K = P_pred / (P_pred + R)
    x_est = x_pred + K * (z - x_pred)
    P = (1 - K) * P_pred

    estimates.append(x_est)

# Create animation
fig, ax = plt.subplots()
ax.set_xlim(0, num_steps)
ax.set_ylim(min(measurements) - 5, max(measurements) + 5)

line1, = ax.plot([], [], 'bo-', label='Measurements')
line2, = ax.plot([], [], 'ro-', label='Kalman Estimates')
line3, = ax.plot([], [], 'g--', label='True Position')
ax.legend()

def update(frame):
    line1.set_data(range(frame + 1), measurements[:frame + 1])
    line2.set_data(range(frame + 1), estimates[:frame + 1])
    line3.set_data(range(frame + 1), true_positions[:frame + 1])
    return line1, line2, line3

ani_single_state = FuncAnimation(fig, update, frames=num_steps, interval=300, blit=True)

# Save the animation
animation_path_single = "kalman_filter_1d_single_state.gif"
ani_single_state.save(animation_path_single, writer=PillowWriter(fps=3))
print(f"Animation saved to: {animation_path_single}")
