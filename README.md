# Double Inverted Pendulum on a Cart (DIPC): Full Control Theory Implementation

A complete, five-stage progression from nonlinear dynamics derivation through Model Predictive Control — implemented entirely in MATLAB/Simulink for a physically parameterized Double Inverted Pendulum on a Cart system. Each stage builds directly on the last, culminating in a real-time MPC controller with Luenberger state estimation running on the nonlinear plant.

> **Purdue University | MS Interdisciplinary Engineering (Autonomy & Robotics) | ECE 680 Modern Automatic Control**

---

## 📋 Table of Contents
- [System Description](#system-description)
- [Project Structure (5 Stages)](#project-structure-5-stages)
  - [Stage 1 — Nonlinear Modeling & Linearization](#stage-1--nonlinear-modeling--linearization)
  - [Stage 2 — LQR State Feedback & Lyapunov Stability](#stage-2--lqr-state-feedback--lyapunov-stability)
  - [Stage 3 — Simulink 3D Simulation](#stage-3--simulink-3d-simulation)
  - [Stage 4 — Model Predictive Control (MPC)](#stage-4--model-predictive-control-mpc)
  - [Stage 5 — Advanced Control](#stage-5--advanced-control)
- [Physical Parameters](#physical-parameters)
- [State Space Definition](#state-space-definition)
- [Dependencies](#dependencies)
- [How to Run](#how-to-run)

---

## System Description

The Double Inverted Pendulum on a Cart (DIPC) is a canonical benchmark in nonlinear control theory — two rigid rods linked in series and balanced vertically on a freely moving cart, controlled by a single horizontal force input. It is inherently unstable at its upright equilibrium and is one of the most demanding underactuated systems in control engineering.

```
         ● m2  (tip mass)
         |
         | l2
         |
         ● m1  (first joint mass)
         |
         | l1
         |
    ┌────┼────┐
    │  Cart M │──── u (force input)
    └─────────┘
    ══════════════  (track)
```

This project progressively designs, verifies, and simulates controllers of increasing sophistication for this system, using the same physical parameters and state-space model throughout.

---

## Physical Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Cart mass | M | 1.5 kg |
| First rod mass | m₁ | 0.5 kg |
| Second rod mass | m₂ | 0.75 kg |
| First rod length | l₁ | 0.5 m |
| Second rod length | l₂ | 0.75 m |
| Gravity | g | 9.81 m/s² |

---

## State Space Definition

The system has 6 states, 1 primary input, and 3 measured outputs:

**State vector:** `x = [x_pos, θ₁, θ₂, ẋ, θ̇₁, θ̇₂]ᵀ`

**Input:** `u = F` (horizontal force on cart) — extended to `[F, τ₁]` in Stage 2 Problem 5

**Output:** `y = [x_pos, θ₁, θ₂]ᵀ`

**Equilibrium:** upright position `x* = 0`, `u* = 0`

---

## Project Structure (5 Stages)

---

### Stage 1 — Nonlinear Modeling & Linearization
**`Funwork1/`**

The entire nonlinear system is derived from first principles and linearized symbolically in MATLAB using the Symbolic Math Toolbox.

**Equations of Motion (Lagrangian mechanics, 3 coupled ODEs):**

```
(M+m₁+m₂)ẍ + (m₁+m₂)l₁θ̈₁cos(θ₁) + m₂l₂θ̈₂cos(θ₂)
    − (m₁+m₂)l₁θ̇₁²sin(θ₁) − m₂l₂θ̇₂²sin(θ₂) = u

(m₁+m₂)l₁²θ̈₁ + (m₁+m₂)l₁ẍcos(θ₁) + m₂l₁l₂θ̈₂cos(θ₁−θ₂)
    + m₂l₁l₂θ̇₂²sin(θ₁−θ₂) − (m₁+m₂)gl₁sin(θ₁) = 0

m₂l₂²θ̈₂ + m₂l₂ẍcos(θ₂) + m₂l₁l₂θ̈₁cos(θ₁−θ₂)
    − m₂l₁l₂θ̇₁²sin(θ₁−θ₂) − m₂gl₂sin(θ₂) = 0
```

**Steps implemented:**
- Symbolic derivation of the full nonlinear EOM using `syms` and `solve()`
- Construction of the state-derivative vector `f(x, u)`
- Linearization at the upright equilibrium via Jacobians: `A = ∂f/∂x|₀`, `B = ∂f/∂u|₀`
- Numerical evaluation of A, B, C, D matrices
- Controllability analysis: `rank(ctrb(A,B)) = 6` → fully controllable
- Observability analysis: `rank(obsv(A,C)) = 6` → fully observable
- Transfer function derivation: `G(s) = C(sI−A)⁻¹B`
- Animated nonlinear simulation using Forward Euler integration with `matlabFunction()` handle
- Video output: `dipc_nonlinear_simulation.mp4`

**Key files:**
```
Funwork1_Gery.m          — Symbolic modeling, linearization, controllability/observability
Funwork1_Gery_NonLinSim.m — Nonlinear open-loop animation (unstable, pendulums fall)
Funwork1_Gery_SimOde.m   — ODE45-based simulation
Funwork1_Gery_Matrix.mat  — Saved A, B, C, D matrices (used by all subsequent stages)
```

---

### Stage 2 — LQR State Feedback & Lyapunov Stability
**`Funwork2/`**

Controller design for the linearized system, Lyapunov stability verification, dual-input extension, and Luenberger observer design — all verified analytically.

**Problems solved:**

**Problem 1 — Open-Loop Lyapunov Analysis:**
Solves the Lyapunov equation `AᵀP + PA = −Q` for the open-loop system. Since the DIPC is unstable, the Lyapunov equation has no positive definite solution, confirming instability.

**Problem 2 — LQR State Feedback Controller:**
Pole placement via conventional `place()` was tested and discarded due to poor performance. The optimal LQR controller was designed instead:
```matlab
Q = diag([1, 10, 10, 0.1, 0.1, 0.1]);  % Penalizes angle deviations heavily
R = 0.1 * eye(1);                        % Moderate control effort penalty
K = lqr(A, B, Q, R);
```
Resulting closed-loop system `Acl = A − BK` verified stable via eigenvalue analysis.

**Problem 3 — Closed-Loop Transfer Function:**
Full transfer function matrix `G_cl(s)` derived for the LQR-controlled system.

**Problem 4 — Closed-Loop Lyapunov Verification:**
Lyapunov equation solved for `Acl` → P is positive definite → asymptotic stability confirmed analytically.

**Problem 5 — Dual-Input Extension (u1, u2):**
System extended with a second input: cart force `u₁` and joint torque `u₂` at the first link. Full re-derivation of EOM with `[u₁, u₂]` input vector, new B matrix (6×2), new LQR design with `R = 0.1·I₂`.

**Problem 6 — Luenberger Observer Design:**
Observer poles placed at half the controller pole values (standard rule of thumb: observer ×2 faster than controller):
```matlab
p_obs = poles_controller / 2;
L = place(A', C', p_obs)';
```
Combined controller-observer compensator verified stable via the 12×12 extended state-space system.

**Problem 7 — Compensator Lyapunov Verification:**
Full `12×12` extended system Lyapunov equation solved to confirm asymptotic stability of the combined controller-observer system.

**Problem 8 — Compensator Transfer Function:**
Minimal realization `minreal()` of the full 3-output × 2-input compensated transfer function matrix.

**Key files:**
```
Funkwork2.m              — Complete 8-problem implementation
Funwork2_Gery_SimOde.m   — Simulation of controlled system
DIPC_Conrolled_Fw2_Sim_Gery.mp4 — Controlled system animation (stable!)
```

---

### Stage 3 — Simulink 3D Simulation
**`Funwork3/`**

Full 3D Simulink model of the closed-loop controlled DIPC system, visually demonstrating stabilization.

**Implementation:**
- Simulink block diagram `DIPC_3DSim.slx` with nonlinear plant, LQR controller, and 3D visualization
- Simulation of controlled vs. uncontrolled behaviour
- Video output: `DIPC_3DSim.mp4` / `DIPC_3DSim.avi`

**Key files:**
```
DIPC_3DSim.slx           — Full Simulink model with 3D animation block
Funwork3.mlx             — Live Script analysis and results
```

---

### Stage 4 — Model Predictive Control (MPC)
**`Funwork4/`**

Three parallel MPC implementations of increasing sophistication, all controlling the nonlinear DIPC plant:

**MPC Version 1 — MATLAB MPC Toolbox (Baseline):**
Uses MATLAB's built-in `mpc()` object with the linearized prediction model. The `mpcmove()` function is called at each timestep to compute the optimal control sequence.

```matlab
% Core MPC loop
for k = 1:t_steps
    y_true = C_u3 * x_true;
    u = mpcmove(mpc_obj, mpc_state, y_true, r_target);
    [T_ode, X_ode] = ode45(@(t,x) f_handle_3u(x, u), [t, t+Ts], x_true);
    x_true = X_ode(end, :)';
end
```
Output: `mpc_sim_P2.mp4`

**MPC Version 2 — MPC with External Luenberger Observer:**
MPC toolbox controller combined with a manually implemented discrete-time Luenberger observer. The observer estimates the full state from partial measurements, and the estimated state deviation `(x̂ − xₑ)` is passed directly to `mpcmove()`:

```matlab
% Observer update (discrete-time)
x_hat = Ad*x_hat + Bd*u + Ld*(y_true - Cd*x_hat);
% MPC uses estimated state
u = mpcmove(mpc_obj, x_hat - xe, y_true, r_target);
```
Output: `mpc_sim_P3.mp4`

**MPC Version 3 — Fully Manual MPC (from scratch):**
The MPC optimization is solved analytically by constructing the prediction matrices manually and computing the gain matrices `Kx` and `Kr` offline. The control law at each timestep is purely a matrix multiply — no optimization solver called at runtime:

```matlab
% Receding horizon law (manual, no toolbox)
delta_u_all = Kr * r_p_dev - Kx * delta_x_hat;
delta_u = delta_u_all(1:m);  % Apply only first move
u = ue + delta_u;
```
This implements the standard unconstrained MPC closed-form solution and demonstrates full understanding of the underlying optimization.
Output: `mpc_sim_P4.mp4`

**Key files:**
```
run_mpc_simulation.m         — MPC Toolbox baseline (P2)
run_mpc_sim_with_Observer.m  — MPC + external Luenberger observer (P3)
run_manual_mpc_sim.m         — Fully manual MPC from scratch (P4)
run_mpc_with_Ld_observer.m   — Observer gain computation utility
Funwork4.mlx                 — Full analysis and parameter sweeps
```

---

### Stage 5 — Advanced Control
**`Funwork5/`**

Extension of the DIPC control framework to advanced optimal and robust control techniques.

**Key files:**
```
Funwork5.mlx             — Advanced control implementation
FW5_DIPC_3DSim.avi       — Simulation output
```

---

## Dependencies

- MATLAB R2023a or later
- Control System Toolbox (`lqr`, `place`, `lyap`, `ss`, `tf`)
- Symbolic Math Toolbox (`syms`, `jacobian`, `solve`, `subs`)
- Model Predictive Control Toolbox (`mpc`, `mpcmove`, `mpcstate`)
- Simulink (for Stage 3)

---

## How to Run

### Stage 1 — Nonlinear Simulation
```matlab
cd Funwork1
run('Funwork1_Gery.m')           % Derives A, B, C, D and saves matrices
run('Funwork1_Gery_NonLinSim.m') % Animates unstable open-loop DIPC
```

### Stage 2 — LQR + Observer
```matlab
cd Funwork2
run('Funkwork2.m')               % Complete 8-problem analysis
```

### Stage 3 — Simulink
```matlab
cd Funwork3
open('DIPC_3DSim.slx')           % Open and run the Simulink model
```

### Stage 4 — MPC
```matlab
cd Funwork4
run('Funwork4.mlx')              % Run full MPC analysis
% Individual simulations called from inside the Live Script
```

---

## Progression Summary

| Stage | Method | Input | Key Concept |
|---|---|---|---|
| FW1 | Nonlinear EOM + Linearization | u (1 input) | Symbolic Jacobians, controllability/observability |
| FW2 | LQR + Lyapunov + Luenberger Observer | u1, u2 (2 inputs) | Optimal state feedback, stability proof |
| FW3 | Simulink 3D Closed-Loop | u1, u2 | Visual verification |
| FW4 | MPC (3 implementations) | u1, u2, u3 (3 inputs) | Predictive control, receding horizon |
| FW5 | Advanced Control | — | Extended optimal/robust methods |

---

## Author

**Phillipp Gery** — Purdue University, MS Interdisciplinary Engineering (Autonomy & Robotics)
Fulbright Scholar | GradBridge Program (Purdue & UC Berkeley)