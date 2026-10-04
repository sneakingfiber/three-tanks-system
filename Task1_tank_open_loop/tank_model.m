% ASSIGNMENT 1 -- TASK 1: Nonlinear Continuous-Time Model
% Open-loop simulation of the 3-tank system

clear; clc;

% 1. System Parameters
params.A1 = 1.2;   % Tank 1 area [m^2]
params.A2 = 1.0;   % Tank 2 area [m^2]
params.A3 = 1.4;   % Tank 3 area [m^2]

params.qin = 0.18; % Inlet flow to Tank 1 [m^3/s]
params.cin = 1.0;  % Inlet oxygen concentration [mg/L]

params.k12 = 0.22; % Hydraulic coefficient T1-T2
params.k23 = 0.20; % Hydraulic coefficient T2-T3

params.ka = 0.9;   % Aeration transfer coefficient
params.csat = 8.0; % Saturation oxygen concentration [mg/L]

params.lambda2 = 0.08; % Oxygen decay in Tank 2 [1/s]
params.lambda3 = 0.05; % Oxygen decay in Tank 3 [1/s]

% 2. Initial Conditions
h1_0 = 2.68; h2_0 = 1.85; h3_0 = 0.85; % Levels [m]
c2_0 = 5.81; c3_0 = 4.48;             % Concentrations [mg/L]
x0 = [h1_0; h2_0; h3_0; c2_0; c3_0];

% 3. Constant Inputs (Open Loop)
ua0 = 0.5;  % Aeration
ur0 = 0.02; % Recycle
ud0 = 0.18; % Discharge
u0 = [ua0; ur0; ud0];

% 4. Simulation Setup
tspan = [0; 1500];
options = odeset('NonNegative', 1:5, 'RelTol', 1e-6);

% 5. Solve ODE
[t, x] = ode45(@(t,x) tank_ode(t, x, u0, params), tspan, x0, options);

% 6. Plotting
figure(1);
subplot(2,1,1);
plot(t, x(:,1), t, x(:,2), t, x(:,3), 'LineWidth', 1.5);
legend('h1','h2','h3'); ylabel('Level [m]'); grid on;
title('Task 1: Open-Loop Tank Levels');

subplot(2,1,2);
plot(t, x(:,4), t, x(:,5), 'LineWidth', 1.5);
legend('c2','c3'); xlabel('Time [s]'); ylabel('Oxygen [mg/L]'); grid on;
title('Task 1: Open-Loop Oxygen Concentration');
