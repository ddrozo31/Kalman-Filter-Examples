import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Time setup
num_steps = 30
dt = 1.0

# True motion (position + velocity)
true_velocity = 1.0
true_position = 0.0
true_states = np.array([[true_position + i * true_velocity, true_velocity] for i in range(num_steps)])

# Simulate noisy measurements of position
measurement_noise_std = 2.0
measurements = true_states[:, 0] + np.random.normal(0, measurement_noise_std, num_steps)

# Initial estimates
x_est = np.array([0.0, 0.0])  # [position, velocity]
P = np.eye(2) * 1000          # Large initial uncertainty

# System matrices
F = np.array([[1, dt], [0, 1]])  # State transition model
H = np.array([[1, 0]])           # Measurement model (measure position only)
Q = np.eye(2) * 0.01             # Process noise
R = np.array([[measurement_noise_std ** 2]])  # Measurement noise

# Store estimates
estimates = []

for z in measurements:
    # Prediction
    x_pred = F @ x_est
    P_pred = F @ P @ F.T + Q

    # Update
    y = z - H @ x_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_est = x_pred + K @ y
    P = (np.eye(2) - K @ H) @ P_pred

    estimates.append(x_est.copy())

# Extract positions for animation
estimated_positions = [e[0] for e in estimates]
true_positions = true_states[:, 0]

# Create animation
fig, ax = plt.subplots()
ax.set_xlim(0, num_steps)
ax.set_ylim(min(measurements) - 5, max(measurements) + 5)

line_meas, = ax.plot([], [], 'bo-', label='Measurements')
line_est, = ax.plot([], [], 'ro-', label='Kalman Estimates')
line_true, = ax.plot([], [], 'g--', label='True Position')
ax.legend()

def update(frame):
    line_meas.set_data(range(frame + 1), measurements[:frame + 1])
    line_est.set_data(range(frame + 1), estimated_positions[:frame + 1])
    line_true.set_data(range(frame + 1), true_positions[:frame + 1])
    return line_meas, line_est, line_true

ani_2d = FuncAnimation(fig, update, frames=num_steps, interval=300, blit=True)

# Save animation
animation_path_2d = "kalman_filter_2d_position_velocity.gif"
ani_2d.save(animation_path_2d, writer=PillowWriter(fps=3))
print(f"Animation saved to: {animation_path_2d}")
