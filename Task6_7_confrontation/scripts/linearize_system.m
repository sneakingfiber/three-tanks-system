% ASSIGNMENT 1 -- TASK 5: Equilibrium Point and Linearization (Symbolic Method)

% 1. Parameters (Numerical)
params.A1 = 1.2; params.A2 = 1.0; params.A3 = 1.4;
params.qin = 0.18; params.cin = 1.0;
params.k12 = 0.22; params.k23 = 0.20;
params.ka = 0.9; params.csat = 8.0;
params.lambda2 = 0.08; params.lambda3 = 0.05;

% 2. Symbolic Definitions
syms h1 h2 h3 c2 c3 ua ur ud real
syms A1 A2 A3 qin cin k12 k23 ka csat lambda2 lambda3 real

% Define symbolic state and input vectors
X = [h1; h2; h3; c2; c3];
U = [ua; ur; ud];

% Nonlinear state equations (f)
q12 = k12 * sqrt(h1 - h2);
q23 = k23 * sqrt(h2 - h3);

dh1 = (1/A1) * (qin + ur - q12);
dh2 = (1/A2) * (q12 - q23);
dh3 = (1/A3) * (q23 - ud - ur);
dc2 = (q12 / (A2 * h2)) * (cin - c2) + ka * ua * (csat - c2) - lambda2 * c2;
dc3 = (q23 / (A3 * h3)) * (c2 - c3) - lambda3 * c3;

f = [dh1; dh2; dh3; dc2; dc3];

% Output equation (y)
y = [h2; h3; c3];

% 3. Calculate Symbolic Jacobians
A_sym = jacobian(f, X);
B_sym = jacobian(f, U);
C_sym = jacobian(y, X);
D_sym = jacobian(y, U);

% 4. Find Equilibrium Point
h3_star = 0.90; 
c3_star = 5.50; 
ur_star = 0.02; 

% Analytical derivation for equilibrium point (to get x* and u*)
ud_star = params.qin;
q_star = params.qin + ur_star;
h2_star = h3_star + (q_star / params.k23)^2;
h1_star = h2_star + (q_star / params.k12)^2;
c2_star = c3_star * (1 + (params.lambda3 * params.A3 * h3_star) / q_star);
ua_star = (params.lambda2 * c2_star - (q_star/(params.A2*h2_star))*(params.cin-c2_star)) / (params.ka * (params.csat - c2_star));

x_star = [h1_star; h2_star; h3_star; c2_star; c3_star];
u_star = [ua_star; ur_star; ud_star];

% 5. Evaluate Jacobians at Equilibrium
% Define the list of parameters for substitution
sym_params = [A1, A2, A3, qin, cin, k12, k23, ka, csat, lambda2, lambda3];
num_params = [params.A1, params.A2, params.A3, params.qin, params.cin, params.k12, params.k23, params.ka, params.csat, params.lambda2, params.lambda3];

% Substitution
A_num = double(subs(A_sym, [X', U', sym_params], [x_star', u_star', num_params]));
B_num = double(subs(B_sym, [X', U', sym_params], [x_star', u_star', num_params]));
C_num = double(subs(C_sym, [X', U', sym_params], [x_star', u_star', num_params]));
D_num = double(subs(D_sym, [X', U', sym_params], [x_star', u_star', num_params]));

% 6. Display Results
fprintf('\n--- System Linearization Results ---\n');
fprintf('Equilibrium Point x*:\n'); disp(x_star');
fprintf('Equilibrium Input u*:\n'); disp(u_star');

disp('Matrix A (Continuous):'); disp(A_num);
disp('Matrix B (Continuous):'); disp(B_num);
disp('Matrix C:'); disp(C_num);
disp('Matrix D:'); disp(D_num);

% Save to workspace variables
A = A_num; B = B_num; C = C_num; D = D_num;

% 7. Discretization
Ts_default = 0.5; 
sys_cont = ss(A, B, C, D);
sys_disc = c2d(sys_cont, Ts_default, 'zoh');

A_disc = sys_disc.A;
B_disc = sys_disc.B;
C_disc = sys_disc.C;
D_disc = sys_disc.D;

save('linear_model.mat', 'A', 'B', 'C', 'D', 'A_disc', 'B_disc', 'C_disc', 'D_disc', 'x_star', 'u_star');
