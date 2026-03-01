clear all
clc

% VAriables
syms M m1 m2 l1 l2 g
syms theta1 theta2 x_pos theta1_dot theta2_dot x_dot
syms theta1_ddot theta2_ddot x_ddot
syms u

% statevector
state_vector = [x_pos; theta1; theta2; x_dot; theta1_dot; theta2_dot];
input_vector = u;


% EOM --> x 
eq1 = (M + m1 + m2) * x_ddot ...
      + (m1 + m2) * l1 * theta1_ddot * cos(theta1) ...
      + m2 * l2 * theta2_ddot * cos(theta2) ...
      - (m1 + m2) * l1 * theta1_dot^2 * sin(theta1) ...
      - m2 * l2 * theta2_dot^2 * sin(theta2) ...
      - u == 0;

% EOM --> theta1 
eq2 = (m1 + m2) * l1^2 * theta1_ddot...
      + (m1 + m2) * l1 * x_ddot * cos(theta1) ...
      + m2 * l1 * l2 * theta2_ddot * cos(theta1 - theta2) ...
      + m2 * l1 * l2 * theta2_dot^2 * sin(theta1 - theta2) ... 
      - (m1 + m2) * g * l1 * sin(theta1) == 0;

% EOM --> theta2 
eq3 = m2 * l2^2 * theta2_ddot ...
      + m2 * l2 * x_ddot * cos(theta2) ...
      + m2 * l1 * l2 * theta1_ddot * cos(theta1 - theta2) ...
      - m2 * l1 * l2 * theta1_dot^2 * sin(theta1 - theta2) ... 
      - m2 * g * l2 * sin(theta2) == 0;

%Solve equations for higthes derviative
eqddot = solve([eq1, eq2, eq3], [x_ddot, theta1_ddot, theta2_ddot]);


% state-derivative vector f(x,u)
f = [x_dot;
     theta1_dot;
     theta2_dot;
     eqddot.x_ddot
     eqddot.theta1_ddot;
     eqddot.theta2_ddot;];

% Step 5: Calculate Jacobians and Substitute Numerical Values
% Define numerical values
M_val = 1.5; 
m1_val = 0.5; 
m2_val = 0.75;
l1_val = 0.5; 
l2_val = 0.75; 
g_val = 9.81;

% Define the equilibrium point (origin)
eq_point_states = [x_pos, theta1, theta2, x_dot, theta1_dot, theta2_dot];
eq_point_values = [0, 0, 0, 0, 0, 0];
eq_input_value = 0;

%Jacobian parcial dervi
A_sym = jacobian(f, state_vector);
B_sym = jacobian(f, input_vector);


% Substitute parameters and equilibrium values
A_params = subs(A_sym, {M, m1, m2, l1, l2, g}, {M_val, m1_val, m2_val, l1_val, l2_val, g_val});
B_params = subs(B_sym, {M, m1, m2, l1, l2, g}, {M_val, m1_val, m2_val, l1_val, l2_val, g_val});

% Define all variables and values to be substituted at the equilibrium point
all_sym_vars = [eq_point_states, u]; % Combine states and input
all_num_vals = [eq_point_values, eq_input_value]; % Combine state values and input value

% Substitute all values into BOTH matrices in one clear step
A_at_eq = subs(A_params, all_sym_vars, all_num_vals);
B_at_eq = subs(B_params, all_sym_vars, all_num_vals);

% Now, convert the fully-substituted symbolic matrices to numerical doubles
A = double(A_at_eq);
B = double(B_at_eq);

% Display the final calculated Jacobian matrices
disp('Jacobian Matrix A:');
disp(A);
disp('Jacobian Matrix B:');
disp(B);

%Add Definition of C and D

C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

D = [0;
     0;
     0];



save("Funwork1_Gery_Matrix", "A", "B", "C", "D")


%Kalmansche controller Form

Qc = [B A*B A^2*B A^3*B A^4*B A^5*B]
rank(Qc)

%Kalmansche Observer form

Qc = [C;
      C*A;
      C*A^2;
      C*A^3;
      C*A^4;
      C*A^5]

rank(Qc);

n = size(A, 1); %Full Rank

%cont
Qc = ctrb(A, B);
rank_Qc = rank(Qc);

fprintf('Number of states (n): %d\n', n);
fprintf('Rank of the controllability matrix: %d\n', rank_Qc);
if rank_Qc == n
    disp('Conclusion: The system is controllable.');
else
    disp('Conclusion: The system is not controllable.');
end

%Observability 
Qo = obsv(A, C);
rank_Qo = rank(Qo);

fprintf('\nRank of the observability matrix: %d\n', rank_Qo);
if rank_Qo == n
    disp('Conclusion: The system is observable.');
else
    disp('Conclusion: The system is not observable.');
end

sys = ss(A, B, C, D);

% Set names for clarity in the output
sys.InputName = 'u';
sys.OutputName = {'x', 'theta1', 'theta2'};

% Compute the transfer function matrix G(s) = C*(sI-A)^-1*B + D
G = tf(sys);

disp('Transfer function from u to x:');
G(1,1) % Access and display the first TF

disp('Transfer function from u to theta1:');
G(2,1) % Access and display the second TF

disp('Transfer function from u to theta2:');
G(3,1) % Access and display the third TF