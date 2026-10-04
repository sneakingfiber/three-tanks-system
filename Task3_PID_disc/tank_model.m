
clear; clc;
% ASSIGNMENT 1 -- TASK 3: Digital PID with ZOH 
% Simulation setup for continuous plant with discrete control

% Params (same as before)
params.A1 = 1.2; params.A2 = 1.0; params.A3 = 1.4;
params.qin = 0.18; params.cin = 1.0;
params.k12 = 0.22; params.k23 = 0.20;
params.ka = 0.9; params.csat = 8.0;
params.lambda2 = 0.08; params.lambda3 = 0.05;

% Sampling setup (Task 3)
Ts = 0.5;   % [s] Initial sampling time
t0 = 0; tf = 150;
time_steps = t0:Ts:tf;

% Initial Conditions (Plant States)
h1_0 = 0.75; h2_0 = 0.70; h3_0 = 0.85;
c2_0 = 4.00; c3_0 = 4.20;
x_plant = [h1_0; h2_0; h3_0; c2_0; c3_0];

% Initial Discrete Controller States (Integrators)
% Initializing with some pre-charge as discussed in GEMINI_md
prev_int_states = [1.0; 1.0; 1.0]; % [Int_h2; Int_h3; Int_c3]

% PID Gains
gains.K_h2 = 0.8; gains.KI_h2 = 0.5;
gains.K_h3 = 0.9; gains.KI_h3 = 1.01;
gains.K_c3 = 0.5; gains.KI_c3 = 1.01;

% Reference signals
ref.h2 = @(t) 0.80 + 0.08*sin(2*pi/60 * t);
ref.h3 = @(t) 0.90 + 0.06*sin(2*pi/90 * t + pi/4);
ref.c3 = @(t) 5.50 + 0.70*(1 - exp(-t/40)) + 0.20*sin(2*pi/75 * t);

% Initialize simulation storage
t_total = [];
x_total = [];
u_total = [];

% Main Simulation Loop (Discrete Control + Continuous Plant)
for k = 1:length(time_steps)-1
    tk = time_steps(k);
    tk_next = time_steps(k+1);
    
    % 1. Measure outputs (ZOH Input)
    y_measured = [x_plant(2); x_plant(3); x_plant(5)]; % h2, h3, c3
    ref_k = [ref.h2(tk); ref.h3(tk); ref.c3(tk)];
    
    % 2. Calculate Discrete Control (Digital PID Logic)
    [u_k, next_int_states] = tank_ode_closed_loop_discrete(y_measured, ref_k, prev_int_states, gains, Ts);
    
    % 3. Run Plant ODE for Ts interval (ZOH Output)
    % Control u_k is held constant during this integration
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t_int, x_int] = ode45(@(t,x) tank_ode(t, x, u_k, params), [tk, tk_next], x_plant, opts);
    
    % 4. Store and Update
    t_total = [t_total; t_int];
    x_total = [x_total; x_int];
    u_total = [u_total; repmat(u_k', length(t_int), 1)];
    
    x_plant = x_int(end, :)'; % Update plant state for next step
    prev_int_states = next_int_states; % Update integrator memory
end

% Plotting results
figure(2);
subplot(3,1,1);
plot(t_total, x_total(:,1), t_total, x_total(:,2), t_total, x_total(:,3), 'LineWidth', 1.5);
hold on;
fplot(ref.h2, [t0, tf], '--r');
fplot(ref.h3, [t0, tf], '--k');
ylabel('Levels [m]'); legend('h1','h2','h3', 'h2_{ref}', 'h3_{ref}');
grid on;

subplot(3,1,2);
plot(t_total, x_total(:,4), t_total, x_total(:,5), 'LineWidth', 1.5);
hold on;
fplot(ref.c3, [t0, tf], '--g');
ylabel('Oxygen [mg/L]'); legend('c2','c3', 'c3_{ref}');
grid on;

subplot(3,1,3);
plot(t_total, u_total(:,1), t_total, u_total(:,2), t_total, u_total(:,3), 'LineWidth', 1.5);
ylabel('Control Inputs'); legend('ua','ur','ud');
xlabel('Time [s]');
grid on;
