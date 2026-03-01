# Double Inverted Pendulum on a Cart — Modern Control Theory

A five-stage progressive study of modern automatic control applied to the Double Inverted Pendulum on a Cart (DIPC). Each Funwork explores one core topic: Lyapunov stability theory, optimal LQR design, LMI-based synthesis with CVX, Model Predictive Control, and Unknown Input Observer design for attack-resilient estimation. The system is extended progressively from 1 input to 3 inputs, and from the origin to a non-trivial non-upright equilibrium.


---

## Physical System

```
         ● m₂  (tip mass)
         |
         | l₂
         |
         ● m₁  (joint mass)
         |
         | l₁
         |
    ┌────┼────┐
    │  Cart M │──── u₁ (cart force)
    └─────────┘     u₂ (torque at joint 1)  ← added in FW2
                    u₃ (torque at joint 2)  ← added in FW3
```

| Parameter | Value |
|---|---|
| Cart mass M | 1.5 kg |
| Link 1 mass m₁ | 0.5 kg |
| Link 2 mass m₂ | 0.75 kg |
| Link 1 length l₁ | 0.5 m |
| Link 2 length l₂ | 0.75 m |
| g | 9.81 m/s² |

**State vector:** `x = [x_pos, θ₁, θ₂, ẋ, θ̇₁, θ̇₂]ᵀ` (6 states)  
**Outputs:** `y = [x_pos, θ₁, θ₂]ᵀ` → C = [I₃ | 0₃], D = 0

---

## Stage Overview

| FW | Topic | Inputs | Key Concept |
|---|---|---|---|
| 1 | Nonlinear Modeling & Linearization | u₁ (1) | Lagrangian mechanics, Jacobian, controllability, observability, TF, animation |
| 2 | Lyapunov Stability Theory & LQR | u₁, u₂ (2) | Lyapunov equation, LQR/ARE, Luenberger observer, 12-state compensator |
| 3 | Equilibrium Analysis & LMI Control | u₁, u₂, u₃ (3) | Non-upright equilibrium, LMI state-feedback + output-feedback via CVX/SDPT3 |
| 4 | Model Predictive Control | u₁, u₂, u₃ (3) | ZOH discretization, constrained MPC, MPC+observer, manual MPC from scratch |
| 5 | Unknown Input Observer (UIO) | u₁, u₂, u₃ (3) | UIO decoupling conditions, LMI observer synthesis, CT+DT, cyber-attack rejection |

---

## FunWork 1 — Nonlinear Modeling & Linearization
**`Funwork1/`** | Single input u₁ (cart force) | Equilibrium: upright origin

**Topic: System Modeling and State-Space Foundations**

The Lagrangian was derived by hand in full (see `FunWork1_Gery_Final.pdf`) and then implemented symbolically in MATLAB.

**Problem 1 — Nonlinear Model:**  
Built the 3 coupled nonlinear EOMs from the Lagrangian L = T − V using Euler-Lagrange equations, one for each generalized coordinate (x, θ₁, θ₂). Solved symbolically for highest-order derivatives [ẍ, θ̈₁, θ̈₂] using `solve()`, then assembled the full nonlinear state-derivative vector `f(x, u)`.

```matlab
% EOM for x (cart)
eq1 = (M+m1+m2)*x_ddot + (m1+m2)*l1*theta1_ddot*cos(theta1) + m2*l2*theta2_ddot*cos(theta2)
      - (m1+m2)*l1*theta1_dot^2*sin(theta1) - m2*l2*theta2_dot^2*sin(theta2) - u == 0

% EOM for theta1 (link 1)
eq2 = (m1+m2)*l1^2*theta1_ddot + (m1+m2)*l1*x_ddot*cos(theta1)
      + m2*l1*l2*theta2_ddot*cos(theta1-theta2) + m2*l1*l2*theta2_dot^2*sin(theta1-theta2)
      - (m1+m2)*g*l1*sin(theta1) == 0

% EOM for theta2 (link 2)
eq3 = m2*l2^2*theta2_ddot + m2*l2*x_ddot*cos(theta2)
      + m2*l1*l2*theta1_ddot*cos(theta1-theta2) - m2*l1*l2*theta1_dot^2*sin(theta1-theta2)
      - m2*g*l2*sin(theta2) == 0
```

**Problem 2 — Linearization via Jacobians:**  
Computed A = ∂f/∂x|₀ and B = ∂f/∂u|₀ at the upright equilibrium (origin).

```
A (linearized):                          B (linearized):
[ 0   0    0     1   0  0 ]              [  0      ]
[ 0   0    0     0   1  0 ]              [  0      ]
[ 0   0    0     0   0  1 ]              [  0      ]
[ 0  -8.175  0   0   0  0 ]              [  0.6667 ]
[ 0  65.4    0   0   0  0 ]              [ -1.3333 ]
[ 0 -32.7   32.7  0  0  0 ]              [  0      ]
```

**Problem 3 — Controllability, Observability & Canonical Forms:**
- `rank(ctrb(A,B)) = 6` → fully controllable
- `rank(obsv(A,C)) = 6` → fully observable
- Transformed to controller companion form and observer companion form using `compreal()`

**Problem 4 — Transfer Functions:**  
Derived all 3 transfer functions G(s) = C(sI−A)⁻¹B:
- G₁₁(s): u → x_pos — 4th-order numerator, 6th-order denominator
- G₂₁(s): u → θ₁ — `(-1.333s² + 43.6) / (s⁴ - 98.1s² + 1176)`
- G₃₁(s): u → θ₂ — `43.6 / (s⁴ - 98.1s² + 1176)` — open-loop unstable (poles at ±9.17, ±3.74)

**Problem 5 — Open-Loop Nonlinear Animation:**  
Simulated the unstable open-loop DIPC using a `matlabFunction()` handle with Forward Euler integration (dt = 0.01 s). Animated with MATLAB handle graphics (cart rectangle, two pendulum links and masses), boundary checks keep the cart in frame. Exported to `dipc_nonlinear_simulation.mp4`.

---

## FunWork 2 — Lyapunov Stability Theory & LQR
**`Funwork2/`** | Inputs: u₁, u₂ (2 inputs) | Equilibrium: origin

**Topic: Lyapunov Stability Theory and Optimal State-Feedback Control**

**Problem 1 — Open-Loop Lyapunov Analysis:**  
Attempted to solve `AᵀP + PA = −Q` for Q = I₆. MATLAB's `lyap()` throws because open-loop eigenvalues are `{0, 0, ±9.17, ±3.74}` — two on the RHP. No positive definite P exists → open-loop system confirmed unstable in the sense of Lyapunov.

**Problem 2 — LQR State-Feedback Controller:**  
Pole placement via `place()` discarded (poor performance). Designed LQR by solving the Algebraic Riccati Equation:

```matlab
Q = diag([1, 10, 10, 0.1, 0.1, 0.1]);  % angles penalized heavily
R = 0.1 * eye(1);
K = lqr(A, B, Q, R);
% Closed-loop poles: {-9.21±0.24i, -3.87±0.85i, -0.75±0.67i} — all LHP
```

**Problem 3 — Closed-Loop Transfer Function:**  
Derived G_cl(s) for the linearized model + LQR. Shared denominator: `s⁶ + 27.67s⁵ + 283.5s⁴ + 1338s³ + 3001s² + 2961s + 1353`.

**Problem 4 — Closed-Loop Lyapunov Verification:**  
Solved `AₒₗᵀP + PAₒₗ = −Q` for A_cl = A − BK. All eigenvalues of P are positive `{0.003, 0.02, 0.27, 0.72, 5.24, 192.1}` → P is positive definite → asymptotic stability proved by Lyapunov's theorem (1892).

**Problem 5 — Dual-Input System (u₁, u₂):**  
Extended nonlinear EOMs to include u₂ (torque at joint 1 as Lagrangian generalized force Q₂). New 6×2 B matrix. Redesigned LQR with `R = 0.1·I₂`. Closed-loop poles now `{-14.56, -10.07, -3.95±1.07i, -0.76±0.69i}`.

**Problem 6 — Luenberger Observer:**  
Observer poles placed at 2× the LQR pole magnitudes (rule of thumb):
```matlab
p_obs = poles_Aclu2 * 2;
L = place(A', C', p_obs)';
% Observer poles: {-29.12, -20.13, -7.90±2.13i, -1.52±1.39i} — all LHP
```

**Problem 7 — 12-State Extended Compensator Lyapunov Verification:**  
Assembled 12×12 extended system matrix A_ex from plant + observer. Solved Lyapunov equation with Q = I₁₂. All 12 eigenvalues of P_ex are positive `{0.007, 0.033, 0.054, ..., 49.64}` → combined compensator is asymptotically stable.

**Problem 8 — Compensator Transfer Function:**  
Built full 12-state extended state-space (A_ex, B_ex, C_ex, D_ex), derived the 3×2 TF matrix, reduced using `minreal()`. Shared denominator: `s⁶ + 34.04s⁵ + 408.2s⁴ + 2146s³ + 5209s² + 5375s + 2588`.

**Problem 9 — Nonlinear Closed-Loop Animation:**  
Simulated combined LQR + observer driving the nonlinear plant via `ode45`. Augmented state `[x_plant; x_hat]` (12 states), zero initial conditions for observer. Exported trajectory data as `timeseries` objects for Simulink 3D animation.

---

## FunWork 3 — Equilibrium Analysis & LMI-Based Control
**`Funwork3/`** | Inputs: u₁, u₂, u₃ (3 inputs) | Non-trivial equilibrium xₑ ≠ 0

**Topic: Linear Matrix Inequalities (LMI) with CVX/SDPT3**

**Problem 1 — No Equilibrium with 1 Input:**  
For `xₑ = [0.1, π/3, π/4, 0, 0, 0]ᵀ`, shows each EOM requires a completely different force to hold still:

| Equation | Required u |
|---|---|
| ẍ = 0 | u = 4.825 N |
| θ̈₁ = 0 | u = 121.530 N |
| θ̈₂ = 0 | u = −8.496 N |

MATLAB solver returns empty set → no single u satisfies all three simultaneously.

**Problem 2 — No Equilibrium with 2 Inputs:**  
Same analysis for `[u₁; u₂]` — solver still returns empty set. Two inputs still insufficient.

**Problem 3 — Equilibrium Found with 3 Inputs:**  
Added u₃ (torque at joint 2 as Lagrangian generalized force Q₃). Solved for equilibrium input:
```
uₑ = [0, −5.3098, −3.9019]ᵀ  at  xₑ = [0.1, π/3, π/4, 0, 0, 0]ᵀ
```

**Problem 4 — Re-Linearization at (xₑ, uₑ):**  
Jacobians re-evaluated at new equilibrium → new A (6×6), B (6×3). Notably different from origin-linearized model because non-zero angles change the linearization.

**Problem 5 — LMI State-Feedback Controller (CVX / SDPT3):**  
Standard change of variables S = P⁻¹, Z = KS converts the stability LMI to a convex SDP:

```matlab
cvx_begin sdp
  variable S(n,n) symmetric
  variable Z(m,n)
  S >= eps*eye(n);
  S*A_u3' + A_u3*S - Z'*B_u3' - B_u3*Z + alpha*S <= -eps*eye(n);  % alpha = 3
cvx_end
K_x = Z / S;
% Closed-loop poles: {-4.87±6.10i, -3.33, -4.07±0.57i, -4.47} — SDPT3, 12 iterations
```

Tested on nonlinear plant with `ode45` and `ode23` — both solvers compared for accuracy and step-size sensitivity.

**Problem 6 — LMI Output-Feedback Controller:**  
LMI in variables (P, N, M) for static output feedback u = −K_o·y. CVX solves in 12 iterations, but all poles land exactly on the imaginary axis `{±0.78i, ±1.61i, ±2.00i}` → purely oscillatory, not asymptotically stable. Proved this is fundamental: the condition `n ≤ m + p − 1` is not satisfied (6 ≤ 3 + 3 − 1 = 5 is false) → no unique stabilizing static output-feedback solution exists for this configuration.

**Problem 7 — Observer-Based State-Feedback vs. Output Feedback:**  
Luenberger observer designed with poles at 3× the LMI controller poles. Side-by-side comparison over t ∈ [0, 5] s:
- **K_x + observer**: asymptotically stable, excellent estimation (`x̂ → x`)
- **K_o output feedback**: oscillates indefinitely (as expected from Problem 6 analysis)

---

## FunWork 4 — Model Predictive Control
**`Funwork4/`** | 3 inputs, non-trivial equilibrium from FW3, discrete-time

**Topic: MPC — augmented model, constrained, observer-based, and manual from scratch**

**Problem 1 — ZOH Discretization:**  
Discretized at h = 0.01 s via Zero-Order Hold using `c2d()`. Resulting Ad (6×6) and Bd (6×3) used as the MPC design model throughout.

**Problem 2 — MPC without Constraints (MATLAB Toolbox, augmented model):**
```matlab
mpc_obj = mpc(sys_d, h, p=100, m=10);
mpc_obj.Weights.OutputVariables     = [2, 20, 20];    % x, θ₁, θ₂
mpc_obj.Weights.ManipulatedVariables = [0.1, 0.1, 0.1];
mpc_obj.Model.Nominal.X = xe_vals;  % non-trivial equilibrium operating point
mpc_obj.Model.Nominal.U = ue;
% MV and OV unconstrained (±Inf)
```
Closed-loop simulation on nonlinear continuous plant via `ode45` at each MPC step. Exported to `mpc_sim_P2.mp4`.

**Problem 3 — MPC with Input and Output Constraints:**
```matlab
mpc_obj.MV(1).Min = -70;  mpc_obj.MV(1).Max = 70;    % cart force [N]
mpc_obj.MV(2).Min = -40;  mpc_obj.MV(2).Max = 40;    % joint torque 1 [Nm]
mpc_obj.MV(3).Min = -40;  mpc_obj.MV(3).Max = 40;    % joint torque 2 [Nm]
mpc_obj.OV(i).Min = -2;   mpc_obj.OV(i).Max = 2;     % all outputs [m or rad]
```
QP solved at each step by the MPC toolbox. Exported to `mpc_sim_P3.mp4`.

**Problem 4 — MPC + Discrete Luenberger Observer:**  
Mapped continuous observer poles to discrete domain (`poles_d = exp(λ_c · h)`). Placed discrete gain Ld via `place()`. Observer eigenvalues: `{0.85±0.16i, 0.90, 0.87, 0.88±0.02i}` — all inside unit circle. Combined with MPC toolbox, plotted true vs. estimated states. Exported to `mpc_sim_P4.mp4`.

**Problem 5 — Manual MPC from Scratch (no toolbox, non-augmented model):**  
Derived the unconstrained MPC closed-form solution directly from lecture notes. Prediction matrices W and Z constructed offline:

```matlab
% W(i*p : ...) = Cd * Ad^i          (lower stacked powers of Ad)
% Z = lower-block-triangular Toeplitz from Cd*Ad^(i-1)*Bd blocks

K_r = [I, 0...] * (R_mpc + Z'*Q_mpc*Z)^-1 * Z'*Q_mpc   % (3 × 300)
K_x = K_r * W                                             % (3 × 6)

% At each step — pure matrix multiply, no QP solver needed:
%   delta_u[k] = K_r * r_preview - K_x * delta_x_hat[k]
%   u_applied[k] = ue + delta_u[k]
```

Combined with discrete Luenberger observer (same Ld from Problem 4). Simulated on nonlinear plant. Exported to `manual_mpc_sim_NONAUG.mp4`.

---

## FunWork 5 — Unknown Input Observer (UIO)
**`Funwork5/`** | 3 inputs, non-trivial equilibrium, cyber-attack scenario

**Topic: UIO Design — Necessary & Sufficient Conditions, LMI Synthesis, Attack Rejection**

**Sections 1.1–1.3 — Theory (hand-derived before code):**  
Starting from the DT model with unknown input d[k] entering through B₂:

- **Condition 1**: Find M such that `M · [C·B₂, D₂] = [B₂, 0]` — decouples unknown actuator input
- **Condition 2**: Resulting transformed state z[k] = (I−MC)·x[k] must also decouple the second unknown channel
- Combined **Necessary & Sufficient condition**: `rank([C·B₂, D_sens]) = rank(B₂) + rank(D_sens)`
- Decoupling matrix: `M = [B₂, 0] · pinv([C·B₂, D_sens])`
- Modified system matrix: `A₁ = (I − M·C) · A`
- Observer exists iff (C, A₁) is detectable → find L such that (A₁ − LC) is Schur stable

LMI formulation: stability condition expressed as Lyapunov inequality → converted to LMI via Schur complement with substitution Z = PL.

**Problem 2 — Continuous-Time UIO (pole placement):**
```matlab
% Extended output: C_new includes original 3 outputs + x_dot sensor (row 4)
% UIO condition: rank([C·B₂, D_sens]) == rank(B₂) + rank(D_sens) → SATISFIED
% → "UIO Condition Satisfied: Can decouple Both Attacks."

K_fb = lqr(A, B, diag([1000,1000,1000,1,1,1]), 0.1);
% ctrl poles: {-29.7±15.1i, -4.2±4.1i, -10.3±9.1i}
desired_obs_poles = 6 * ctrl_poles;
L_uio = place(A1', C', desired_obs_poles)';
% |obs poles| ~ 10² magnitude
```

Simulated with simultaneous attacks on nonlinear plant via `ode45`:
- **Actuator attack**: step +5 N on u₁ for t ∈ [3, 4] s
- **Sensor attack**: bias +0.1 on x_pos sensor intermittently (every 3 s, duration 0.1 s)

State estimates converge; system stabilizes throughout both attack windows.

**Problem 3 — Discrete-Time UIO via LMI (CVX / SDPT3):**
```matlab
% Observer gain synthesis — LMI with exponential decay rate alpha = 0.5
cvx_begin sdp quiet
  variable P(n,n) symmetric
  variable Z(n, p)
  P >= eye(n)*1e-4;
  [-P, Omega21_T'; Omega21_T, Omega22] <= 0;
  % Omega21_T = [P*A1 - Z*C, -Z*D2]
cvx_end
L_uio = P \ Z;
% |eig(A1 - L_uio*C)| = {0.32, 0.58, 0.035, 0, 0, 0} — all inside unit circle
```

Discrete controller also designed via LMI (CVX, spectral radius bound ρ = 0.99):
```matlab
% Controller poles: {0.940, 0.989±0.008i, 0.975, 0.972, 0.965} — inside unit circle
```

Full **hybrid simulation** (discrete UIO + LMI controller, nonlinear plant via `ode15s`):
- **Actuator attack**: u₁ += 10 N for t ∈ [1.5, 2.0] s
- **Sensor attack**: y₁ += 0.2 for t ∈ [4.0, 5.5] s

State estimates converge; controller maintains stability throughout both attack windows.

---

## Project Structure

```
DIPC_Control_Design/
├── README.md
├── .gitignore
├── Funwork1/
│   ├── Funwork1_Gery.m              ← Symbolic EOMs, linearization, TF, controllability/observability
│   ├── Funwork1_Gery_NonLinSim.m    ← Open-loop nonlinear animation (Forward Euler + handle graphics)
│   ├── Funwork1_Gery_SimOde.m       ← ODE45-based simulation utility
│   ├── SimpPendulumAni2.m           ← Animation utility
│   └── FunWork1_Gery_Final.pdf      ← Full hand-derived Lagrangian (PDF)
├── Funwork2/
│   ├── Funkwork2.m                  ← Lyapunov, LQR, 2-input extension, observer, 12-state system (9 problems)
│   └── Funwork2_Gery_SimOde.m       ← Closed-loop nonlinear simulation
├── Funwork3/
│   ├── Funwork3.mlx                 ← Equilibrium analysis, LMI (state + output feedback), ode45 vs ode23
│   └── DIPC_3DSim.slx               ← Simulink 3D closed-loop model
├── Funwork4/
│   ├── Funwork4.mlx                 ← MPC design and analysis (5 problems)
│   ├── run_mpc_simulation.m         ← MPC Toolbox simulation runner (P2, P3)
│   ├── run_mpc_sim_with_Observer.m  ← MPC + discrete Luenberger observer (P4)
│   ├── run_manual_mpc_sim.m         ← Manual MPC + observer — no toolbox (P5)
│   └── run_mpc_with_Ld_observer.m   ← Observer utility
└── Funwork5/
    └── Funwork5.mlx                 ← CT UIO (pole placement) + DT UIO (LMI via CVX) + attack simulations
```

---

## Dependencies

| Toolbox | Used in |
|---|---|
| Symbolic Math Toolbox | FW1, FW2, FW3 — `syms`, `jacobian`, `solve`, `subs` |
| Control System Toolbox | FW1–FW4 — `lqr`, `place`, `lyap`, `ss`, `c2d`, `tf`, `compreal` |
| CVX with SDPT3 solver | FW3 — LMI state-feedback and output-feedback; FW5 — LMI UIO + controller |
| MPC Toolbox | FW4 P2–P4 — `mpc`, `mpcmove` |
| Simulink | FW3 — 3D animation |

---

## Key Technical Highlights

**FW2 — Lyapunov-backed stability proofs:** Every stability claim is backed by solving the Lyapunov matrix equation numerically, not just checking eigenvalues. Both the 1-input LQR closed loop and the full 12-state controller-observer compensator are verified this way.

**FW3 — LMI output feedback limitation:** Rather than stopping at a marginal result, the failure of static output feedback is explained analytically. The condition `n ≤ m + p − 1` (6 ≤ 5) is not satisfied, so no unique asymptotically stabilizing solution exists — the LMI correctly returns poles on the imaginary axis.

**FW4 — Manual MPC without any toolbox:** Prediction matrices W (300×6) and K_r (3×300) are computed offline from the system matrices. The receding-horizon control law at runtime is a single matrix multiply — no QP solver involved. This demonstrates genuine understanding of the MPC cost function and its closed-form solution.

**FW5 — Simultaneous actuator + sensor attack rejection:** Both an actuator disturbance and a sensor bias are injected simultaneously in the same simulation run. The UIO decouples the unknown input channel while the LMI-based gain ensures exponential convergence of the estimation error, validated on the nonlinear plant.

---

## Author

**Phillipp Gery** | Purdue University, MS Interdisciplinary Engineering (Autonomy & Robotics)  
Fulbright Scholar | GradBridge Program (Purdue & UC Berkeley)