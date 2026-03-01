
clear all
clc


%% Funwork2

%Start with the non-linear model of the double inverted pendulum on a cart (DIPC) from FunWork 1.

%Phillipp Gery

syms theta1 theta2 x_pos theta1_dot theta2_dot x_dot
syms theta1_ddot theta2_ddot x_ddot
syms u


%get Results from Funwork1 means Model 
load("../Funwork1/Funwork1_Gery_Results.mat");
disp("non-linear model")
disp(f)

%Jacobians and insert Numerical Values
% Define the equilibrium point (origin) again and Liarize model around Eq
% Point
eq_point_states = [x_pos, theta1, theta2, x_dot, theta1_dot, theta2_dot];
eq_point_values = [0, 0, 0, 0, 0, 0];
eq_input_value = 0;
%Jacobian parcial dervi
A_sym = jacobian(f, state_vector);
B_sym = jacobian(f, input_vector);
% variables and values  at the eq. point
all_sym_vars = [eq_point_states, u]; % states and input
all_num_vals = [eq_point_values, eq_input_value]; 
% Substitute all left values
A_at_eq = subs(A_sym, all_sym_vars, all_num_vals)
B_at_eq = subs(B_sym, all_sym_vars, all_num_vals)
% to numerical doubles
A = double(A_at_eq);
B = double(B_at_eq);

%% Problem 1.

%Solve the Lyapunov matrix equation for the linearized open-loop system. Take .
% Is the open-loop system asymptotically stable, that is, is the equilibrium state
% of interest asymptotically stable in the sense of Lyapunov? Explain.

n = size(A,1);

Q = eye(n)

try
% Solve  Lyapunov  A'*P + P*A = -Q
P = lyap(A', Q);

% if P is positive definite
% P is positive definite if and only if all its eigenvalues are positive.
eigenvalues_of_P = eig(P)
catch
   eig_A = eig(A)
   disp("Eigenvalues of A are not all on the open left Plain so the System is not Asym Stable")
end




%% Problem 2. 
% (4 pts) Design a linear state-feedback controller for the linearized model.

% BAd performance with convential Poleplacement 
% Pol  = pole(sys)
% p = ones(1,6)*-2
% p = [-2, -2.1, -2.2, -2.3, -2.4, -2.5]
% p = [-6, -6.1, -6.2, -6.3, -6.4, -6.5]
% K = place(A,B,p);

% --> use Riccaty Controller
% Define LQR weighting matrices Q and R
% Q penalizes state errors. We want to keep the angles small.
Q = diag([1, 10, 10, 0.1, 0.1, 0.1]);
% R holds down contoller effort
R = 0.1 * eye(1);
% Calculate K using LQR
K = lqr(A, B, Q, R);

%new System Matrix with Controller 
Acl = A-B*K;
syscl = ss(Acl,B,C,D);
Pcl = pole(syscl)

%step(syscl)
% bode(syscl)
% nyquist(syscl)

%% Problem 3. 
% (4 pts) Find the transfer function of the closed-loop system composed of the linearized model driven by the linear state-feedback controller.


G_cl = tf(syscl)

disp('Transfer function from u to x:');
G_cl(1,1) % first TF

disp('Transfer function from u to theta1:');
G_cl(2,1) % second TF

disp('Transfer function from u to theta2:');
G_cl(3,1) %  third TF


%% Problem 4. 
% (4 pts) Construct a Lyapunov function for the closed-loop system comprised of the linearized model driven by the  state-feedback controller, solve the Lyapunov matrix equation, 
% and check if the equilibrium state of interest of the closed-loop system is asymptotically stable in the sense of Lyapunov.

%Theorem 4.1 (A. M. Lyapunov, 1892) The system x ̇ = Ax, x(t0) = x0,
% is asymptoti- cally stable if and only if for any given real symmetric positive definite (r.s.p.d.) matrix
%Q the solution P to the continuous Lyapunov matrix equation,AT P + P A = − Q (4.6)
% is also (real symmetric) positive definite.

try
% Solve  Lyapunov  A'*P + P*A = -Q
P = lyap(Acl', Q);

% P is positive definite if and only if all its eigenvalues are positive.
eigenvalues_of_P = eig(P)
catch
   eig_A = eig(Acl)
   disp("Eigenvalues of Acl are not all on the open left Plain so the System is not Asym Stable")
end

if all(eigenvalues_of_P > 0)
    disp('Conclusion: P is positive definite. The closed-loop system is ASYMPTOTICALLY STABLE in the sense of Lyapunov.');
else
    disp('Conclusion: P is NOT positive definite. The closed-loop system is NOT asymptotically stable in the sense of Lyapunov. ');
end


%% Problem 5. 
% (4 pts) Add an extra input in the double inverted pendulum on a cart (DIPC) non-linear model from the previous FunWork,
% namely u1 & u2 , the torque at the first joint. where  is the force applied to the cart and
% is the torque applied at the first joint. In summary, we have now a three-output two-input system.

% Design a stabilizing state-feedback controller using the linearized model. 
% where  is the force applied to the cart and  is the torque applied at the first joint.
% In summary, we have now a three-output two-input system.
% Design a stabilizing state-feedback controller using the linearized model.


% add sys for the new input and params because again without subs
syms u1 u2
syms M m1 m2 l1 l2 g
%Values of parmas
M_val = 1.5; 
m1_val = 0.5;
m2_val = 0.75;
l1_val = 0.5;
l2_val = 0.75;
g_val = 9.81;
%input vector update
input_vector = [u1; u2];


% Because u2 is the tourqe on the first stick ist Q_2 of the Lagraian EoM
% --> nessary to Update direct chnage in the nonlinear Model hard because
% of all the simplifications in the equation

% EOM --> x with u1
eq1 = (M + m1 + m2) * x_ddot ...
      + (m1 + m2) * l1 * theta1_ddot * cos(theta1) ...
      + m2 * l2 * theta2_ddot * cos(theta2) ...
      - (m1 + m2) * l1 * theta1_dot^2 * sin(theta1) ...
      - m2 * l2 * theta2_dot^2 * sin(theta2) ...
      - u1 == 0

% EOM --> theta1 with added u2
eq2 = (m1 + m2) * l1^2 * theta1_ddot...
      + (m1 + m2) * l1 * x_ddot * cos(theta1) ...
      + m2 * l1 * l2 * theta2_ddot * cos(theta1 - theta2) ...
      + m2 * l1 * l2 * theta2_dot^2 * sin(theta1 - theta2) ... 
      - (m1 + m2) * g * l1 * sin(theta1)- u2 == 0

% EOM --> theta2 
eq3 = m2 * l2^2 * theta2_ddot ...
      + m2 * l2 * x_ddot * cos(theta2) ...
      + m2 * l1 * l2 * theta1_ddot * cos(theta1 - theta2) ...
      - m2 * l1 * l2 * theta1_dot^2 * sin(theta1 - theta2) ... 
      - m2 * g * l2 * sin(theta2) == 0

%perform again Linarization as done in Funwork1 
% copy code
%Solve equations for higthes derviative
eqddot = solve([eq1, eq2, eq3], [x_ddot, theta1_ddot, theta2_ddot]);
% nonlinear System Model f(x,u)
f_u2 = [x_dot;
     theta1_dot;
     theta2_dot;
     eqddot.x_ddot
     eqddot.theta1_ddot;
     eqddot.theta2_ddot;];

%sub into f
f_u2 = simplify(subs(f_u2, {M, m1, m2, l1, l2, g}, {M_val, m1_val, m2_val, l1_val, l2_val, g_val}));

%Jacobians and insert Numerical Values
% Define the equilibrium point (origin) again and Liarize model around Eq
% Point
eq_point_states = [x_pos, theta1, theta2, x_dot, theta1_dot, theta2_dot];
eq_point_values = [0, 0, 0, 0, 0, 0];
%new Input Vector with 2 time = 0 --> Optimal case no force from outside
eq_input_value = [0 0];
%Jacobian parcial dervi
A_sym = jacobian(f_u2, state_vector);
B_sym = jacobian(f_u2, input_vector);
% variables and values  at the eq. point
all_sym_vars = [eq_point_states, input_vector.']; % states and input
all_num_vals = [eq_point_values, eq_input_value]; 
% Substitute all left values
A_at_eq = subs(A_sym, all_sym_vars, all_num_vals);
B_at_eq = subs(B_sym, all_sym_vars, all_num_vals);
% to numerical doubles
A_u2 = double(A_at_eq);
B_u2 = double(B_at_eq);
disp('new Jacobian Matrix A (Linarized) for System with 2*u:');
disp(A_u2); % Stays teh sam because the System rains the same only additional Input
disp('new Jacobian Matrix B (Linarized) for System with 2*u:');
disp(B_u2);
disp('Matrix C for System with 2*u stay s the same:');
C_u2 = C;
disp(C_u2)
disp('new Matrix D  for System with 2*u:');
D_u2 = zeros(3, 2);
disp(D_u2)


% Controller Design Again

% --> use Riccaty Controller AGAIN for new Controller
% Define LQR weighting matrices Q and R
% Q penalizes state errors. We want to keep the angles small.
Q_u2 = diag([1, 10, 10, 0.1, 0.1, 0.1]);
% R penalizes control effort.
R_u2 = 0.1 * eye(2); % thsi time eye(2) because we have 2 inputs
% Calculate K using LQR
K_u2 = lqr(A_u2, B_u2, Q_u2, R_u2);


% NEW closed-loop system matrix
A_clu2 = A_u2 - B_u2*K_u2;

% Find the poles (eigenvalues) of the closed-loop system to verify stability
disp("Poles of the System with 2 Inputs and new Controller")
poles_Aclu2 = eig(A_clu2)

% Check stability
if all(real(poles_Aclu2) < 0)
    disp('Conclusion: All poles have negative real parts. The closed-loop system is ASYMPTOTICALLY STABLE.');
else
    disp('Conclusion: At least one pole has a non-negative real part. The system is NOT stable.');
end


%% Problem 6. (4 pts) Use the linearized model to design a Luenberger observer

%define the Poles 
% Rule of the Thump
% oberserver Poles *2 = Controller Poles
p_obs = poles_Aclu2/2;

% Calculate the observer  matrix L 
L = place(A', C', p_obs)'

% Verify stability
A_obs = A - L*C;
observer_poles = eig(A_obs)

if max(real(observer_poles)) < 0
    disp('Conclusion: All observer poles are in the left-half plane. The observer is stabe!!');
else
    disp('Conclusion: The observer is not stable.');
end


% Extended StateSpace Model

%%% TEST TEST TEST TEST TEST
F_ex = eye(6)

A_ex = [A_clu2 -B_u2*K_u2;
        L*C_u2 A_clu2-B_u2*K_u2-L*C_u2]

B_ex = [B_u2;
       B_u2]

C_ex = [C_u2 -D_u2*K_u2]

D_ex = [D_u2]




%% Problem 7. 
% (4 pts) Construct a Lyapunov function for the closed-loop system composed of the linearized model
% driven by the combined controller-observer compensator,
% solve the Lyapunov matrix equation, using  and check if the equilibrium state of interest of the closed-loop system
% is asymptotically stable in the sense of Lyapunov.


Q_ex = eye(12);

try
% Solve  Lyapunov  A'*P + P*A = -Q
P_ex = lyap(A_ex', Q_ex);

% P is positive definite if and only if all its eigenvalues are positive.
eig_of_P_ex = eig(P_ex)
catch
   eig_A_ex = eig(A_ex)
   if all(eig_A_ex > 0)
        disp("Eigenvalues of A_ex are not all on the open left Plain so the System is not Asym Stable")
   end
end

if all(eig_of_P_ex > 0)
    disp('Conclusion: P_ex is positive definite. The closed-loop system is ASYMPTOTICALLY STABLE in the sense of Lyapunov.');
else
    disp('Conclusion: P_ex is NOT positive definite. The closed-loop system is NOT asymptotically stable in the sense of Lyapunov. ');
end


%% Problem 8. 
% (4 pts) Find the transfer function of the closed-loop system composed of the linearized model,
% driven by the combined controller-observer compensator.


sys_obs = ss(A_ex,B_ex, C_ex , D_ex);
sys_obs.InputName = {'u1', 'u2'};
sys_obs.OutputName = {'x', 'theta1', 'theta2'};

% transfer function G(s) = C*(sI-A)^-1*B + D 
G_obs = tf(sys_obs);

%simplify transfer function
G_obssimp = minreal(G_obs);

disp('Transfer function from u1 to x:');
G_obssimp(1,1) 
disp('Transfer function from u1 to theta1:');
G_obssimp(2,1) 
disp('Transfer function from u1 to theta2:');
G_obssimp(3,1) 
disp('Transfer function from u2 to x:');
G_obssimp(1,2)
disp('Transfer function from u2 to theta1:');
G_obssimp(2,2) 
disp('Transfer function from u2 to theta2:');
G_obssimp(3,2) 

























