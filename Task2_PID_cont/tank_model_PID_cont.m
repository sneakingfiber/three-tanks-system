% ASSIGNMENT 1 -- TASK 2: Continuous-Time PID Control
% Closed-loop simulation using continuous PI regulators

clear; clc;

% 1. System Parameters
params.A1 = 1.2; params.A2 = 1.0; params.A3 = 1.4;
params.qin = 0.18; params.cin = 1.0;
params.k12 = 0.22; params.k23 = 0.20;
params.ka = 0.9; params.csat = 8.0;
params.lambda2 = 0.08; params.lambda3 = 0.05;

% 2. Initial Conditions
h1_0 = 0.68; h2_0 = 0.85; h3_0 = 0.85;
c2_0 = 5.81; c3_0 = 4.48;

% PID Integrator Initial States (Pre-charge)
Int_h2_0 = 1.0; Int_h3_0 = 1.0; Int_c3_0 = 1.0;

% Combined Initial State Vector
x0 = [h1_0; h2_0; h3_0; c2_0; c3_0; Int_h3_0; Int_c3_0; Int_h2_0];

% 3. Reference Signal Definitions
ref.h2 = @(t) 0.80 + 0.08*sin(2*pi/60 * t);
ref.h3 = @(t) 0.90 + 0.06*sin(2*pi/90 * t + pi/4);
ref.c3 = @(t) 5.50 + 0.70*(1 - exp(-t/40)) + 0.20*sin(2*pi/75 * t);

% 4. Equilibrium / Feedforward Points
eq.h3 = 0.85; eq.c3 = 4.48; eq.h2 = 0.85;
eq.ud = 0.18; eq.ua = 0.5; eq.ur = 0.02;

% 5. PID Controller Gains
gains.K_h3 = 0.9;  gains.KI_h3 = 1.01; % Discharge loop
gains.K_c3 = 0.5;  gains.KI_c3 = 1.01; % Aeration loop
gains.K_h2 = 0.8;  gains.KI_h2 = 0.5;  % Recycle loop

% 6. Simulation Setup
tspan = [0; 1500];
options = odeset('NonNegative', 1:5, 'RelTol', 1e-6);

% 7. Solve ODE (Closed Loop)
[t, x] = ode45(@(t,x) tank_ode_closed_loop(t, x, params, eq, ref, gains), tspan, x0, options);

% 8. Plotting
figure(1);
subplot(2,1,1);
plot(t, x(:,1), t, x(:,2), t, x(:,3), 'LineWidth', 1.5);
hold on; fplot(ref.h2, [0, 1500], '--r'); fplot(ref.h3, [0, 1500], '--k');
legend('h1','h2','h3', 'h2_{ref}', 'h3_{ref}'); ylabel('Level [m]'); grid on;
title('Task 2: Continuous-Time PID Control - Levels');

subplot(2,1,2);
plot(t, x(:,4), t, x(:,5), 'LineWidth', 1.5);
hold on; fplot(ref.c3, [0, 1500], '--g');
legend('c2','c3', 'c3_{ref}'); xlabel('Time [s]'); ylabel('Oxygen [mg/L]'); grid on;
title('Task 2: Continuous-Time PID Control - Oxygen');
