function [u, next_states] = tank_ode_closed_loop_discrete(y, ref_k, prev_states, gains, Ts)
% Explicit Discrete-time PID Control Law (Task 3)
% y: current measured outputs [h2; h3; c3]
% ref_k: current references [h2_ref; h3_ref; c3_ref]
% prev_states: previous integral states [Int_h2; Int_h3; Int_c3]
% Ts: sampling time (0.5s)

% 1. Extract measurements and references
h2 = y(1); h3 = y(2); c3 = y(3);
h2_ref = ref_k(1); h3_ref = ref_k(2); c3_ref = ref_k(3);

% 2. Calculate Tracking Errors
e_h2 = h2_ref - h2;
e_h3 = h3 - h3_ref; % Note: inverted for ud because higher ud decreases h3
e_c3 = c3_ref - c3;

% 3. Proportional Terms (P)
P_h2 = gains.K_h2 * e_h2;
P_h3 = gains.K_h3 * e_h3;
P_c3 = gains.K_c3 * e_c3;

% 4. Integral Update (Forward Euler: I(k) = I(k-1) + Ki*Ts*e(k))
Int_h2_new = prev_states(1) + gains.KI_h2 * Ts * e_h2;
Int_h3_new = prev_states(2) + gains.KI_h3 * Ts * e_h3;
Int_c3_new = prev_states(3) + gains.KI_c3 * Ts * e_c3;

% 5. Control Law Assembly (P + I)
ur = P_h2 + Int_h2_new;
ud = P_h3 + Int_h3_new;
ua = P_c3 + Int_c3_new;

% 6. Saturations (ZOH Output)
ur = min(max(ur, 0), 0.08);
ud = min(max(ud, 0.05), 0.25);
ua = min(max(ua, 0), 1);

u = [ua; ur; ud];
next_states = [Int_h2_new; Int_h3_new; Int_c3_new];

end
