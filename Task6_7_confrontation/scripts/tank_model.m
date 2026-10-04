% Tank Model Parameters and Initial Conditions
% Used for system initialization

% 1. System Parameters
params.A1 = 1.2; params.A2 = 1.0; params.A3 = 1.4;
params.qin = 0.18; params.cin = 1.0;
params.k12 = 0.22; params.k23 = 0.20;
params.ka = 0.9; params.csat = 8.0;
params.lambda2 = 0.08; params.lambda3 = 0.05;

% 2. Initial Conditions (Plant States)
h1_0 = 0.75; h2_0 = 0.70; h3_0 = 0.85;
c2_0 = 4.00; c3_0 = 4.20;

% 3. Reference signal definitions
ref.h2 = @(t) 0.80 + 0.08*sin(2*pi/60 * t);
ref.h3 = @(t) 0.90 + 0.06*sin(2*pi/90 * t + pi/4);
ref.c3 = @(t) 5.50 + 0.70*(1 - exp(-t/40)) + 0.20*sin(2*pi/75 * t);
