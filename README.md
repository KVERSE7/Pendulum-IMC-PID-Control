# Inverted Pendulum Stabilization using IMC-Tuned PID Control

![Python](https://img.shields.io/badge/Python-3.8+-blue?logo=python&logoColor=white)
![Gymnasium](https://img.shields.io/badge/Gymnasium-Pendulum--v1-green)
![Control](https://img.shields.io/badge/Control-IMC%20PID-orange)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen)
![Course](https://img.shields.io/badge/Course-ECE3112-lightgrey)

> A hybrid control system that swings up and stabilizes an inverted pendulum using **Energy-Based Swing-Up Control** + **IMC-Tuned PID Stabilization**, simulated in the OpenAI Gymnasium `Pendulum-v1` environment.

---

## Table of Contents

- [Overview](#overview)
- [Theory](#theory)
  - [System Model](#system-model)
  - [Swing-Up Control](#swing-up-control)
  - [IMC-Based PID Tuning](#imc-based-pid-tuning)
  - [Gain Scheduling](#gain-scheduling)
- [Results](#results)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Course Info](#course-info)

---

## Overview

The inverted pendulum is a classic benchmark problem in nonlinear control. The system is inherently **unstable at the upright position** and requires a two-phase strategy:

1. **Swing-Up Phase** — Inject energy to pump the pendulum from the resting (downward) position toward the upright equilibrium.
2. **Stabilization Phase** — Switch to a PID controller to hold the pendulum at the unstable upright equilibrium.

PID gains are derived analytically using **Internal Model Control (IMC)** theory, with a single tuning parameter `λ` controlling the trade-off between speed and stability.

---

## Theory

### System Model

The pendulum dynamics are described by:

$$I\ddot{\theta} = u - mg\frac{l}{2}\sin(\theta)$$

With physical parameters used in simulation:

| Parameter | Value |
|-----------|-------|
| Mass `m` | 2.5 kg |
| Length `l` | 1.5 m |
| Gravity `g` | 9.81 m/s² |
| Moment of Inertia `I` | 1.875 kg·m² |
| Gravitational Constant `Kg` | 36.78 N·m/rad |

---

### Swing-Up Control

Total mechanical energy of the pendulum:

$$E = \frac{1}{2}I\dot{\theta}^2 + mg\frac{l}{2}\cos(\theta)$$

Target energy at the upright position:

$$E_{target} = mg\frac{l}{2}$$

The swing-up controller injects torque proportional to the energy error, gradually pumping the pendulum toward the upright region.

---

### IMC-Based PID Tuning

The linearized transfer function near upright equilibrium:

$$G_p(s) = \frac{e^{-sT}}{Is^2 + K_g}$$

Using IMC design with Padé approximation for the delay term, the PID gains are derived as:

$$K_p = K_g + \frac{I}{\lambda^2}, \quad K_d = \frac{2I}{\lambda}, \quad K_i = 0.6$$

Where **λ** is the IMC closed-loop time constant — the single tuning knob of the entire system.

---

### Gain Scheduling

The controller automatically switches modes based on pendulum angle:

| Condition | Mode |
|-----------|------|
| `\|θ\| < 0.6 rad` | PID Stabilization |
| `\|θ\| > 1.0 rad` | Swing-Up |

---

## Results

Five values of λ were tested. Summary of behaviour:

| λ | Kp | Kd | Behaviour |
|---|----|----|-----------|
| 0.1 | 205.89 | 37.50 | Very aggressive, actuator saturates, fast but oscillatory |
| 0.2 | 65.27 | 18.75 | Fast, slightly smoother, some saturation |
| 0.4 | 30.11 | 9.375 | ✅ Best trade-off — smooth swing-up, stable hold |
| 0.6 | 23.60 | 6.25 | Stable, slower settling, minimal oscillations |
| 0.8 | 21.32 | 4.69 | Very smooth, slowest response, no saturation |

**λ = 0.4** provides the optimal balance between settling speed, control effort, and stability.

---

## Project Structure

```
inverted-pendulum-imc-pid/
│
├── controller.py       # IMC auto-tuning + PID & swing-up logic
├── simulation.py       # Gymnasium Pendulum-v1 wrapper
├── main.py             # Run simulation, log data, generate plots
├── report.pdf          # Full project report (ECE3112)
└── README.md
```

---

## Installation

```bash
# Clone the repo
git clone https://github.com/KVERSE7/inverted-pendulum-imc-pid.git
cd inverted-pendulum-imc-pid

# Install dependencies
pip install gymnasium numpy matplotlib pygame
```

---

## Usage

```bash
python main.py
```

The simulation runs for 800 steps (40 seconds). A live render window opens showing the pendulum. Press any key or close the window to stop early.

**To change λ**, edit `main.py`:

```python
LAMBDA = 0.4   # try 0.1, 0.2, 0.4, 0.6, 0.8
```

After the simulation, three plots are generated automatically:
- **Trajectory** — angle θ over time
- **Control Effort** — torque signal (Nm)
- **Total Energy** — vs target energy

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `gymnasium` | Pendulum-v1 physics simulation |
| `numpy` | Numerical computation |
| `matplotlib` | Plotting results |
| `pygame` | Render window + keyboard events |


---

## Author

**Kartik Sharma** — 23UEC558  
B.Tech ECE, LNMIIT Jaipur  
[GitHub](https://github.com/KVERSE7) · [LinkedIn](https://www.linkedin.com/in/kartik-sharma-lnmiit/)
