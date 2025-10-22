import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Time settings
num_steps = 30
dt = 1.0

# True motion (nonlinear dynamics)
true_state = np.array([0.0, 1.0])  # [position, velocity]
true_states = []

for _ in range(num_steps):
    x, v = true_state
    x_new = x + v * np.cos(x) * dt
    v_new = v
    true_state = np.array([x_new, v_new])
    true_states.append(true_state.copy())

true_states = np.array(true_states)

# Simulate noisy position measurements
measurement_noise_std = 1.5
measurements = true_states[:, 0] + np.random.normal(0, measurement_noise_std, num_steps)

# EKF initialization
x_est = np.array([0.0, 0.0])  # [position, velocity]
P = np.eye(2) * 1000
Q = np.eye(2) * 0.01
R = np.array([[measurement_noise_std ** 2]])

# Measurement model
H = np.array([[1, 0]])

# Store estimates
estimates = []

for z in measurements:
    # Prediction (nonlinear model)
    x, v = x_est
    x_pred = np.array([
        x + v * np.cos(x) * dt,
        v
    ])

    # Jacobian of f(x)
    F = np.array([
        [1 - v * np.sin(x) * dt, np.cos(x) * dt],
        [0, 1]
    ])

    P_pred = F @ P @ F.T + Q

    # Update
    y = z - x_pred[0]
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_est = x_pred + K.flatten() * y
    P = (np.eye(2) - K @ H) @ P_pred

    estimates.append(x_est.copy())

# Extract data for animation
true_positions = true_states[:, 0]
estimated_positions = [e[0] for e in estimates]

# Create animation
fig, ax = plt.subplots()
ax.set_xlim(0, num_steps)
ax.set_ylim(min(measurements) - 5, max(measurements) + 5)

line_meas, = ax.plot([], [], 'bo-', label='Measurements')
line_est, = ax.plot([], [], 'ro-', label='EKF Estimates')
line_true, = ax.plot([], [], 'g--', label='True Position')
ax.legend()

def update(frame):
    line_meas.set_data(range(frame + 1), measurements[:frame + 1])
    line_est.set_data(range(frame + 1), estimated_positions[:frame + 1])
    line_true.set_data(range(frame + 1), true_positions[:frame + 1])
    return line_meas, line_est, line_true

ani_nonlinear = FuncAnimation(fig, update, frames=num_steps, interval=300, blit=True)

# Save animation
animation_path_nonlinear = "ekf_nonlinear_dynamics.gif"
ani_nonlinear.save(animation_path_nonlinear, writer=PillowWriter(fps=3))
print(f"Animation saved to: {animation_path_nonlinear}")
