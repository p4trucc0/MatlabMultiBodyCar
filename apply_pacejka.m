function [Fx, Fy, Mz, SR, SA] = apply_pacejka(Fz, phi, p, s, d, h, l0, l1, rr, xc_1, yc_1, zc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, omega, tyre_param)

% Calculating absolute speed of virtual contact point 24/5/2020 - 3
% phi_fl = 0.1; % TODO: move away.
x_M_Oc_c_0 = [p; s + d; h - l0 - rr]; % position of hub w.r. to car
x_M_Oc_c_1 = [0; 0; -l1];
X_M_Oc_c = v2M(x_M_Oc_c_0)'; 
A_c = generate_a(rho_0, beta_0, sigma_0);
L_c_0 = generate_jacobian(rho_0, beta_0, sigma_0);
L_t_c = [-cos(phi) -sin(phi) 0; -sin(phi) cos(phi) 0; 0 0 -1];
q_theta_c1 = [rho_1; beta_1; sigma_1];
q_lin_c1 = [xc_1; yc_1; zc_1];
vp_0 = [q_lin_c1 + L_c_0*X_M_Oc_c*A_c*q_theta_c1 + L_c_0*x_M_Oc_c_1];
vp_t = L_t_c \ vp_0;
w_free_rolling = -vp_t(1) / rr;
if abs(w_free_rolling) > 0
    slip_rate = 100*(omega - w_free_rolling) / w_free_rolling;
else
    slip_rate = 0; % TODO: handle this, otherwise it'll be impossible to start from standing
end
vp_tm_0_refspeed = L_c_0 * [-rr*1*cos(phi); -rr*1*sin(phi); 0]; % speed component of the tyre, reprojected in absolute ref system with ref speed
vp_tm_0 = L_c_0 * [-rr*omega*cos(phi); -rr*omega*sin(phi); 0]; % speed component of the tyre, reprojected in absolute ref system
vp_wtm_0 = vp_0 + vp_tm_0; % actual speed of the contact point.
slip_angle = atan2(vp_0(2), vp_0(1)) - atan2(vp_tm_0_refspeed(2), vp_tm_0_refspeed(1));
slip_angle_deg = 180 - rad2deg(slip_angle); % TODO: consider correcting this subtracting 180...
while slip_angle_deg > 180
    slip_angle_deg = slip_angle_deg - 360;
end
while slip_angle_deg < -180
    slip_angle_deg = slip_angle_deg + 360;
end
% keyboard
% apply force to wheel and update angular speed.
Fz_pac = max([(Fz / 1000) 0]);
if Fz_pac == 0
    Fx = 0; Fy = 0; Mz = 0;
else
    [Fx, Fy, Mz] = pacejka96(tyre_param, Fz_pac, 0.0, slip_rate, slip_angle_deg);
end

SR = slip_rate;
SA = slip_angle_deg;