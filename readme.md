# Kalman Filter Examples

This repository contains a collection of **Kalman Filter (KF)** and **Extended Kalman Filter (EKF)** simulation scripts implemented in Python. Each example illustrates state estimation for dynamic systems, along with animations showing the convergence and accuracy of the filter.

---

## Repository Structure

| File | Description |
|------|--------------|
| `kalman_filter_1D_static_system.py` | Demonstrates a basic Kalman Filter for a 1D system with constant state (static model). |
| `kalman_filter_1D_dynamic_system.py` | Implements a Kalman Filter for a 1D dynamic system with motion updates. |
| `kalman_filter_2state.py` | Simulates a 2-state Kalman Filter estimating both position and velocity. |
| `ekf_nonlinear_dynamics.py` | Example of an **Extended Kalman Filter (EKF)** applied to a nonlinear dynamic system. |
| `*.gif` | Animated visualizations of each simulation showing the estimation process and convergence. |

---

## Requirements

This repository uses standard scientific Python libraries.  
You can install them with:

```bash
pip install numpy matplotlib
```

---

## Usage

Clone this repository and run any of the example scripts:

```bash
git clone https://github.com/your-username/EKF_Ex.git
cd EKF_Ex/EKF_Ex
python kalman_filter_1D_dynamic_system.py
```

Each script will:
1. Simulate a true system trajectory.  
2. Generate noisy measurements.  
3. Apply the Kalman Filter or EKF.  
4. Display and save an animation (`.gif`) illustrating the estimation process.

---

## Results

The repository includes pre-generated `.gif` animations such as:

- `kalman_filter_1d_animation.gif` – shows state convergence for a 1D motion system.  
- `kalman_filter_2d_position_velocity.gif` – demonstrates position and velocity tracking.  
- `ekf_nonlinear_dynamics.gif` – illustrates EKF performance on nonlinear dynamics.

---

## Concepts Covered

- Kalman Filter equations for linear systems  
- Extended Kalman Filter for nonlinear models  
- Process and measurement noise modeling  
- Visualization of estimation error and covariance  
- Animation of convergence and filter behavior

---

## Mathematical Background

### 1. Linear Kalman Filter (KF)

The Kalman Filter estimates the system state $x_k$ for a linear dynamic model:

$$
x_k = A_k x_{k-1} + B_k u_k + w_k
$$
$$
z_k = H_k x_k + v_k
$$

where:  
- $x_k$ is the state vector  
- $u_k$ is the control input  
- $z_k$ is the measurement  
- $A_k, B_k, H_k$ are system matrices  
- $w_k$ and $v_k$ are zero-mean Gaussian noises with covariances $Q_k$ and $R_k$

#### **Prediction step**
$$
\hat{x}_{k|k-1} = A_k \hat{x}_{k-1|k-1} + B_k u_k
$$
$$
P_{k|k-1} = A_k P_{k-1|k-1} A_k^T + Q_k
$$

#### **Update step**
$$
K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
$$
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H_k \hat{x}_{k|k-1})
$$
$$
P_{k|k} = (I - K_k H_k) P_{k|k-1}
$$

---

### 2. Extended Kalman Filter (EKF)

For **nonlinear** systems described by:

$$
x_k = f(x_{k-1}, u_k) + w_k
$$
$$
z_k = h(x_k) + v_k
$$

The EKF linearizes the nonlinear functions around the current estimate using the Jacobians:

$$
F_k = \frac{\partial f}{\partial x}\Big|_{\hat{x}_{k-1|k-1}}, \quad H_k = \frac{\partial h}{\partial x}\Big|_{\hat{x}_{k|k-1}}
$$

#### **Prediction**
$$
\hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_k)
$$
$$
P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k
$$

#### **Update**
$$
K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
$$
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k [z_k - \hat{x}_{k|k-1}]
$$
$$
P_{k|k} = (I - K_k H_k) P_{k|k-1}
$$

---

## Educational Purpose

These examples are designed for students and researchers learning about:
- Probabilistic estimation and sensor fusion
- Linear vs. nonlinear filtering
- Error covariance and uncertainty propagation
- Implementation of KF/EKF algorithms in Python

---

## Author

Developed by *Professor*: David Rozo-Osorio, I.M. M.Sc. email: david.rozo31@eia.edu.co

**EIA University**, Mechatronical Eng. - Industrial Robotics. For research and teaching purposes in **control, robotics, and estimation systems**.
