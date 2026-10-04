function x_next = tank_ode_discrete(t, x, u, params, Ts)
% TANK_ODE_DISCRETE Plant state transition using Forward Euler
% x(k+1) = x(k) + Ts * f(x(k), u(k))

% 1. Calculate continuous-time derivatives
dx = tank_ode(t, x, u, params);

% 2. Apply Forward Euler step
x_next = x + Ts * dx;

% 3. Safety: ensure states remain non-negative
x_next = max(x_next, 0); 

end
