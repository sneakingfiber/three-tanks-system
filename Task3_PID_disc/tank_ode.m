function dxdt = tank_ode(t, x, u, params)
% function args are: time, states, inputs, system params
% 3-tank cascade model function
% Those are the function that describe a physical model
% representing 3 tanks linked in cascade

% States

h1 = max(x(1), 1e-3);
h2 = max(x(2), 1e-3);
h3 = max(x(3), 1e-3);

c2 = max(x(4), 0);
c3 = max(x(5), 0);

% Inputs
ua = u(1);
ur = u(2);
ud = u(3);

% Prameters
A1 = params.A1;
A2 = params.A2;
A3 = params.A3;

qin = params.qin;
cin = params.cin;

k12 = params.k12;
k23 = params.k23;

ka = params.ka;
csat = params.csat;

lambda2 = params.lambda2;
lambda3 = params.lambda3;

% Flows 
q12 = k12 * sqrt(max(h1 - h2, 0));
q23 = k23 * sqrt(max(h2 - h3, 0));
q12 = min(q12, h1*A1);
q23 = min(q23, h2*A2);  %NO negative flows

% Equations

dh1 = 1/A1 * ((qin + ur)-(q12));
dh2 = 1/A2 * (q12 - q23);
dh3 = 1/A3 * (q23 - (ud+ur));
dc2 = (q12/(A2*h2) * (cin - c2) + ka*ua*(csat-c2)-lambda2*c2);
dc3 = ((q23/(A3*h3) * (c2 - c3)) - lambda3*c3);

% Prevent negative levels

if x(1) <= 0 && dh1 < 0
    dh1 = 0;
end

if x(2) <= 0 && dh2 < 0
    dh2 = 0;
end

if x(3) <= 0 && dh3 < 0
    dh3 = 0;
end

%output

dxdt = [dh1; dh2; dh3; dc2; dc3]; 

end

