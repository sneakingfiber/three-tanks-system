function dxdt = tank_ode_closed_loop(t, x, params, eq, ref, gains)
% TANK_ODE_CLOSED_LOOP Augmented ODE function including PID states
% Integrates plant dynamics and controller integral states

% 1. Extract States
% Plant: h1, h2, h3, c2, c3
% Controller Integrators: Int_h3, Int_c3, Int_h2
h3 = x(3); c3 = x(5); h2 = x(2);
Int_h3 = x(6); Int_c3 = x(7); Int_h2 = x(8);

% 2. Calculate Tracking Errors
eh3 = h3 - ref.h3(t); % Inverted error for discharge pump (ud)
ec3 = ref.c3(t) - c3; % Standard error for aeration (ua)
eh2 = ref.h2(t) - h2; % Standard error for recycle (ur)

% 3. PI Control Laws
ud = gains.K_h3 * eh3 + gains.KI_h3 * Int_h3;
ua = gains.K_c3 * ec3 + gains.KI_c3 * Int_c3;
ur = gains.K_h2 * eh2 + gains.KI_h2 * Int_h2;

% 4. Actuator Saturations
ud = min(max(ud, 0.05), 0.25);
ua = min(max(ua, 0), 1);
ur = min(max(ur, 0), 0.08);

% 5. Plant Dynamics
u = [ua; ur; ud];
dx_plant = tank_ode(t, x(1:5), u, params);

% 6. Integrator Dynamics (Derivatives of I-states)
dInt_h3 = eh3;
dInt_c3 = ec3;
dInt_h2 = eh2;

dxdt = [dx_plant; dInt_h3; dInt_c3; dInt_h2];

end
