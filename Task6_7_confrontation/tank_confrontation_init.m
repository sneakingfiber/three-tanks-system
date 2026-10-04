clear; clc;
% TANK_CONFRONTATION_INIT
% Centralized initialization for Simulink comparison (Task 6 & 7)

% 1. SAMPLING TIME CONFIGURATION
% Change this value to study the effect of discretization (Task 7)
sample_time = 2.0; 
Ts = sample_time; 

% 2. LOAD SYSTEM PARAMETERS AND INITIAL CONDITIONS
% Parameters are defined in the scripts/ folder
run('scripts/tank_model.m');

% 3. RUN LINEARIZATION
% This script calculates the continuous matrices and the equilibrium point
run('scripts/linearize_system.m');

% 4. DISCRETIZATION OF THE LINEAR MODEL
% Ensuring the discrete matrices match the current sample_time
sys_cont = ss(A, B, C, D);
sys_disc = c2d(sys_cont, sample_time, 'zoh');

A_disc = sys_disc.A;
B_disc = sys_disc.B;
C_disc = sys_disc.C;
D_disc = sys_disc.D;

% 5. DEFINE EQUILIBRIUM VECTORS FOR SIMULINK
% Target outputs for summation blocks: y = [h2; h3; c3]
y_star = [x_star(2); x_star(3); x_star(5)];

% 6. PID GAINS (Digital Tuning)
gains.Kp_h3 = 0.8; gains.Ki_h3 = 0.1;
gains.Kp_h2 = 0.2; gains.Ki_h2 = 0.05;
gains.Kp_c3 = 0.1; gains.Ki_c3 = 0.05;

% 7. INITIAL CONDITIONS FOR SIMULINK BLOCKS
% Physics-based initial states
h1_init = h1_0;
h2_init = h2_0;
h3_init = h3_0;
c2_init = c2_0;
c3_init = c3_0;

% Initial state for the State-Space block (in variations)
% delta_x(0) = x(0) - x_star
x_init_linear = [h1_0; h2_0; h3_0; c2_0; c3_0] - x_star;

% PID integrator pre-charge values
Int_h3_0 = 1.0; 
Int_c3_0 = 1.0; 
Int_h2_0 = 1.0;

fprintf('\n--- Initialization Complete (Task 6 & 7) ---\n');
fprintf('Current Sample Time: %.2f s\n', sample_time);
fprintf('All variables loaded into Workspace.\n');
