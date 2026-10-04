function dxdt = tank_ode(t, x, u, params)
% TANK_ODE 3-tank cascade physical model
% Describes liquid levels and oxygen concentration dynamics

% 1. States and Protections
h1 = max(x(1), 1e-3);
h2 = max(x(2), 1e-3);
h3 = max(x(3), 1e-3);
c2 = max(x(4), 0);
c3 = max(x(5), 0);

% 2. Control Inputs
ua = u(1); ur = u(2); ud = u(3);

% 3. Hydraulic Flows (Torricelli's Law)
q12 = params.k12 * sqrt(max(h1 - h2, 0));
q23 = params.k23 * sqrt(max(h2 - h3, 0));

% Prevent outflow exceeding available volume
q12 = min(q12, h1 * params.A1);
q23 = min(q23, h2 * params.A2);

% 4. System Equations
eps_h = 1e-4; % Numerical stability protection

% Mass Balances
dh1 = (1/params.A1) * (params.qin + ur - q12);
dh2 = (1/params.A2) * (q12 - q23);
dh3 = (1/params.A3) * (q23 - ud - ur);

% Oxygen Balances
dc2 = (q12 / (params.A2 * max(h2, eps_h))) * (params.cin - c2) + ...
      params.ka * ua * (params.csat - c2) - params.lambda2 * c2;
dc3 = (q23 / (params.A3 * max(h3, eps_h))) * (c2 - c3) - params.lambda3 * c3;

% Hard protection against negative levels
if x(1) <= 0 && dh1 < 0, dh1 = 0; end
if x(2) <= 0 && dh2 < 0, dh2 = 0; end
if x(3) <= 0 && dh3 < 0, dh3 = 0; end

dxdt = [dh1; dh2; dh3; dc2; dc3]; 

end
