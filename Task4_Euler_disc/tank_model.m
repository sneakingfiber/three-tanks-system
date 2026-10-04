% ASSIGNMENT 1 -- TASK 4: Euler Discretization of the Nonlinear Model
% Full discrete-time simulation (Plant + Controller)

clear; clc;

% 1. SAMPLING CONFIGURATION
sample_time = 0.5;   % Sampling time [s]
Ts = sample_time; 

% 2. System Parameters
params.A1 = 1.2; params.A2 = 1.0; params.A3 = 1.4;
params.qin = 0.18; params.cin = 1.0;
params.k12 = 0.22; params.k23 = 0.20;
params.ka = 0.9; params.csat = 8.0;
params.lambda2 = 0.08; params.lambda3 = 0.05;

% 3. Initial Conditions (Plant States)
h1_0 = 0.75; h2_0 = 0.70; h3_0 = 0.85;
c2_0 = 4.00; c3_0 = 4.20;
x_plant_0 = [h1_0; h2_0; h3_0; c2_0; c3_0];

% Explicit variables for Simulink Discrete Integrator blocks
h1_init = h1_0;
h2_init = h2_0;
h3_init = h3_0;
c2_init = c2_0;
c3_init = c3_0;

% 4. PID Integrator Initial States (Pre-charge)
Int_h2_0 = 1.0; Int_h3_0 = 1.0; Int_c3_0 = 1.0;

% 5. PID Controller Gains
gains.K_h2 = 0.8; gains.KI_h2 = 0.5;
gains.K_h3 = 0.9; gains.KI_h3 = 1.01;
gains.K_c3 = 0.5; gains.KI_c3 = 1.01;

% 6. Reference Signal Definitions
ref.h2 = @(t) 0.80 + 0.08*sin(2*pi/60 * t);
ref.h3 = @(t) 0.90 + 0.06*sin(2*pi/90 * t + pi/4);
ref.c3 = @(t) 5.50 + 0.70*(1 - exp(-t/40)) + 0.20*sin(2*pi/75 * t);

% 7. Simulation Setup
t0 = 0; tf = 150;
t = t0:Ts:tf;
N = length(t);
x = zeros(5, N);
x(:,1) = x_plant_0;
u_total = zeros(N,3);
t_total = t';

% Initialize integrator states for the simulation loop
I_h2 = Int_h2_0; I_h3 = Int_h3_0; I_c3 = Int_c3_0;

% 8. Main Discrete-Time Simulation Loop (Forward Euler)
for k = 1:N-1
    xk = x(:,k);
    tk = t(k);
    
    % Measurements and Tracking Errors
    h2_ref_k = ref.h2(tk); h3_ref_k = ref.h3(tk); c3_ref_k = ref.c3(tk);
    e_h2 = h2_ref_k - xk(2);
    e_h3 = xk(3) - h3_ref_k; % Inverted error for ud
    e_c3 = c3_ref_k - xk(5);
    
    % PID Integral Update (Forward Euler)
    I_h2 = I_h2 + Ts * e_h2;
    I_h3 = I_h3 + Ts * e_h3;
    I_c3 = I_c3 + Ts * e_c3;
    
    % Control Laws
    ua = gains.K_c3 * e_c3 + gains.KI_c3 * I_c3;
    ur = gains.K_h2 * e_h2 + gains.KI_h2 * I_h2;
    ud = gains.K_h3 * e_h3 + gains.KI_h3 * I_h3;
    
    % Actuator Saturations
    ua = min(max(ua, 0), 1);
    ur = min(max(ur, 0), 0.08);
    ud = min(max(ud, 0.05), 0.25);
    u = [ua; ur; ud];
    
    % Plant Evolution (Euler Discretization step)
    dx = tank_ode(tk, xk, u, params);
    x(:,k+1) = xk + Ts * dx;
    
    % Numerical Stability Protection
    x(:,k+1) = max(x(:,k+1), 0);
    u_total(k,:) = u';
end

fprintf('\n--- Task 4 Initialization Complete ---\n');
fprintf('Simulink Variables: sample_time, h1_init, h2_init, h3_init, c2_init, c3_init\n');

% 9. Plotting Results
x_total = x';
figure(1);
subplot(3,1,1);
plot(t_total, x_total(:,1), t_total, x_total(:,2), t_total, x_total(:,3), 'LineWidth', 1.5);
hold on; fplot(ref.h2, [t0, tf], '--r'); fplot(ref.h3, [t0, tf], '--k');
ylabel('Levels [m]'); legend('h1','h2','h3', 'h2_{ref}', 'h3_{ref}'); grid on;
title('Task 4: Euler Discretized Plant Simulation');

subplot(3,1,2);
plot(t_total, x_total(:,4), t_total, x_total(:,5), 'LineWidth', 1.5);
hold on; fplot(ref.c3, [t0, tf], '--g');
ylabel('Oxygen [mg/L]'); legend('c2','c3', 'c3_{ref}'); grid on;

subplot(3,1,3);
stairs(t_total, u_total(:,1)); hold on;
stairs(t_total, u_total(:,2)); stairs(t_total, u_total(:,3));
ylabel('Control Inputs'); legend('ua','ur','ud'); xlabel('Time [s]'); grid on;
