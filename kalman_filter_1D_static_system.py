#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Simulated sensor measurements (noisy observations of a constant position)
true_position = 2.0
num_steps = 50
measurement_noise_std = 0.5
measurements = np.random.normal(true_position, measurement_noise_std, num_steps)

# Kalman filter initialization
x_est = 0.0       # Initial estimate
P = 1.0           # Initial uncertainty
R = 0.5           # Measurement noise variance
Q = 0.0           # Process noise (none, static system)

# Store values for animation
x_estimates = []
P_estimates = []
kalman_gains = []

# Run Kalman filter
for z in measurements:
    # Prediction step (skipped for static system)

    # Update step
    K = P / (P + R)
    x_est = x_est + K * (z - x_est)
    P = (1 - K) * P

    x_estimates.append(x_est)
    P_estimates.append(P)
    kalman_gains.append(K)

# Setup the plot for animation
fig, ax = plt.subplots()
ax.set_xlim(0, num_steps)
ax.set_ylim(min(measurements) - 1, max(measurements) + 1)
ax.axhline(y=true_position, color='green', linestyle='--', label='True Position')

line1, = ax.plot([], [], 'bo-', label='Measurements')
line2, = ax.plot([], [], 'ro-', label='Estimates')
text_annotation = ax.text(0.02, 0.95, '', transform=ax.transAxes)

ax.legend()

def update(frame):
    line1.set_data(range(frame + 1), measurements[:frame + 1])
    line2.set_data(range(frame + 1), x_estimates[:frame + 1])
    text_annotation.set_text(f'Kalman Gain: {kalman_gains[frame]:.2f}')
    return line1, line2, text_annotation

ani = FuncAnimation(fig, update, frames=num_steps, interval=300, blit=True)

# Save the animation as a GIF
animation_path = "kalman_filter_1d_animation.gif"
ani.save(animation_path, writer=PillowWriter(fps=3))
print(f"Animation saved to: {animation_path}")
