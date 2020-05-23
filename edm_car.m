function [xd, par_out] = edm_car(t, y)
% ODE describing a 10-DOF car model.
% x is a 28-element array containing first speeds and then positions.

par_out = struct();

xc_1 = y(1);
yc_1 = y(2);
zc_1 = y(3);
rho_1 = y(4);
beta_1 = y(5);
sigma_1 = y(6);
l_fl_1 = y(7);
l_fr_1 = y(8);
l_rl_1 = y(9);
l_rr_1 = y(10);
theta_fl_1 = y(11);
theta_fr_1 = y(12);
theta_rl_1 = y(13);
theta_rr_1 = y(14);
xc_0 = y(15);
yc_0 = y(16);
zc_0 = y(17);
rho_0 = y(18);
beta_0 = y(19);
sigma_0 = y(20);
l_fl_0 = y(21);
l_fr_0 = y(22);
l_rl_0 = y(23);
l_rr_0 = y(24);
theta_fl_0 = y(25);
theta_fr_0 = y(26);
theta_rl_0 = y(27);
theta_rr_0 = y(28);

q_0 = y(15:24);
q_1 = y(1:10);


% Forces at road - tyre interface
if t >= 2 && t < 2.5
    Fx_fl = 0; Fy_fl = 0;
    Fx_fr = 0; Fy_fr = 0;
    Fx_rl = 0; Fy_rl = 0; %1000*sin(2*pi*t);
    Fx_rr = 0; Fy_rr = 0; %-1000*sin(2*pi*t);
    % Terrain speed
    zpn_fl_0 = .2;
    zpn_fl_1 = 0;
    zpn_fr_0 = .2;
    zpn_fr_1 = 0;
    zpn_rl_0 = 0;
    zpn_rl_1 = 0;
    zpn_rr_0 = 0;
    zpn_rr_1 = 0;
elseif t >= 4 && t < 4.5
    Fx_fl = 0; Fy_fl = 0;
    Fx_fr = 0; Fy_fr = 0;
    Fx_rl = 0; Fy_rl = 0; %1000*sin(2*pi*t);
    Fx_rr = 0; Fy_rr = 0; %-1000*sin(2*pi*t);
    % Terrain speed
    zpn_fl_0 = 0;
    zpn_fl_1 = 0;
    zpn_fr_0 = 0;
    zpn_fr_1 = 0;
    zpn_rl_0 = .2;
    zpn_rl_1 = 0;
    zpn_rr_0 = .2;
    zpn_rr_1 = 0;
elseif t >= 6 && t < 6.5
    Fx_fl = 0; Fy_fl = 0;
    Fx_fr = 0; Fy_fr = 0;
    Fx_rl = 0; Fy_rl = 0; %1000*sin(2*pi*t);
    Fx_rr = 0; Fy_rr = 0; %-1000*sin(2*pi*t);
    % Terrain speed
    zpn_fl_0 = .2;
    zpn_fl_1 = 0;
    zpn_fr_0 = 0;
    zpn_fr_1 = 0;
    zpn_rl_0 = .2;
    zpn_rl_1 = 0;
    zpn_rr_0 = 0;
    zpn_rr_1 = 0;
elseif t >= 8 && t < 8.5
    Fx_fl = 0; Fy_fl = 0;
    Fx_fr = 0; Fy_fr = 0;
    Fx_rl = 0; Fy_rl = 0; %1000*sin(2*pi*t);
    Fx_rr = 0; Fy_rr = 0; %-1000*sin(2*pi*t);
    % Terrain speed
    zpn_fl_0 = 0;
    zpn_fl_1 = 0;
    zpn_fr_0 = .2;
    zpn_fr_1 = 0;
    zpn_rl_0 = 0;
    zpn_rl_1 = 0;
    zpn_rr_0 = .2;
    zpn_rr_1 = 0;
else
    Fx_fl = 0; Fy_fl = 0;
    Fx_fr = 0; Fy_fr = 0;
    Fx_rl = 0; Fy_rl = 0;
    Fx_rr = 0; Fy_rr = 0;
    % Terrain speed
    zpn_fl_0 = 0;
    zpn_fl_1 = 0;
    zpn_fr_0 = 0;
    zpn_fr_1 = 0;
    zpn_rl_0 = 0;
    zpn_rl_1 = 0;
    zpn_rr_0 = 0;
    zpn_rr_1 = 0;
end

% Physical parameters of the model. 
% car body center of gravity position.
mc = 1000; % kg.
Ixc = 300; %kg * m
Iyc = 2000; %kg * m
Izc = 2000; %kg * m
xb = 0.3; %m, distance of COG from car-centered RS in car RS.
yb = 0.0; %m
zb = 1.0; %m

% Suspension geometry
p_f = 1.4; %m, distance of front axle from car RS
s_f = 0.6; %m, lateral distance front wheels
d_f = 0.1; %m, distance of wheel central plane from suspension mount point.
h_f = 0.8; %m, height of suspension mount point.
l_f_ind = 0.8; %m, undeformed suspension length
ks_f = 30000; % N/m, suspension spring
kp_f = 120000; % N/m, tyre as spring
rs_f = 10000; % N*s/m, damper
rp_f = 10000; % N*s/m, tyre as damper
r_f_ind = 0.35;

p_r = 1.4;
s_r = 0.6;
d_r = 0.1;
h_r = 0.8;
ks_r = 30000; % N/m, suspension spring
l_r_ind = 0.8; %m, undeformed suspension length
kp_r = 120000; % N/m, tyre as spring
rs_r = 10000; % N*s/m, damper
rp_r = 10000; % N*s/m, tyre as damper
r_r_ind = 0.35;

mr_f = 20; %kg, wheel + part of suspension + brake
mr_r = 20; 
Jr_f = 10; % radial inertia of front wheel
Jr_r = 10; % radial inertia of rear wheel

% Transforming from absolute distances into proper coordinates.
p_fl =  p_f; s_fl =  s_f; d_fl =  d_f; h_fl = h_f; mr_fl = mr_f; l_fl_ind = l_f_ind;
p_fr =  p_f; s_fr = -s_f; d_fr = -d_f; h_fr = h_f; mr_fr = mr_f; l_fr_ind = l_f_ind;
p_rl = -p_r; s_rl =  s_r; d_rl =  d_r; h_rl = h_r; mr_rl = mr_r; l_rl_ind = l_r_ind;
p_rr = -p_r; s_rr = -s_r; d_rr = -d_r; h_rr = h_r; mr_rr = mr_r; l_rr_ind = l_r_ind;

% Assigning stiffness and damping to each spring/damper.
ks_fl = ks_f; rs_fl = rs_f; kp_fl = kp_f; rp_fl = rp_f; r_fl_ind = r_f_ind;
ks_fr = ks_f; rs_fr = rs_f; kp_fr = kp_f; rp_fr = rp_f; r_fr_ind = r_f_ind;
ks_rl = ks_r; rs_rl = rs_r; kp_rl = kp_r; rp_rl = rp_r; r_rl_ind = r_r_ind;
ks_rr = ks_r; rs_rr = rs_r; kp_rr = kp_r; rp_rr = rp_r; r_rr_ind = r_r_ind;

%% Mass matrices and terms.
% Car body, from "3/5/20 - 7"
L_c_0 = generate_jacobian(rho_0, beta_0, sigma_0);
X_BO_c = v2M([xb; yb; zb])'; % transposed to see whether it corrects roll problem.
A_c = generate_a(rho_0, beta_0, sigma_0);
Lmc_12 = L_c_0 * X_BO_c * A_c;
L_c_qc = [eye(3)       Lmc_12;
         zeros(3)       A_c];
L_c_qt = [L_c_qc       zeros(6, 4)];
M_car_phys = diag([mc mc mc Ixc Iyc Izc]);
M_c = L_c_qt'*M_car_phys*L_c_qt; % in total coordinates.

% Wheels, as a point mass. From "2/5/20 - 4"
% generate_wheel_jacobian(rho, beta, sigma, p, s, d, h, l)
L_r_fl_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_fl, s_fl, d_fl, h_fl, l_fl_0);
L_r_fr_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_fr, s_fr, d_fr, h_fr, l_fr_0);
L_r_rl_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_rl, s_rl, d_rl, h_rl, l_rl_0);
L_r_rr_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_rr, s_rr, d_rr, h_rr, l_rr_0);

L_r_fl_qt = [L_r_fl_qc(:, 1:6) zeros(3,0) L_r_fl_qc(:, 7) zeros(3,3)];
L_r_fr_qt = [L_r_fr_qc(:, 1:6) zeros(3,1) L_r_fr_qc(:, 7) zeros(3,2)];
L_r_rl_qt = [L_r_rl_qc(:, 1:6) zeros(3,2) L_r_rl_qc(:, 7) zeros(3,1)];
L_r_rr_qt = [L_r_rr_qc(:, 1:6) zeros(3,3) L_r_rr_qc(:, 7) zeros(3,0)];

M_r_fl_phys = mr_fl*eye(3);
M_r_fr_phys = mr_fr*eye(3);
M_r_rl_phys = mr_rl*eye(3);
M_r_rr_phys = mr_rr*eye(3);

M_r_fl = L_r_fl_qt'*M_r_fl_phys*L_r_fl_qt;
M_r_fr = L_r_fr_qt'*M_r_fr_phys*L_r_fr_qt;
M_r_rl = L_r_rl_qt'*M_r_rl_phys*L_r_rl_qt;
M_r_rr = L_r_rr_qt'*M_r_rr_phys*L_r_rr_qt;

% Total Mass Matrix
M = M_c + M_r_fl + M_r_fr + M_r_rl + M_r_rr;

% Derivative of Mass Matrix (approximated form)
% from "8/5/20 - 5"
% generate_approx_M_derivative(m, sigma_0, sigma_1, xu_0, yu_0, zu_0, zu_1)
M1_c_q6 = generate_approx_M_derivative(mc, sigma_0, sigma_1, xb, yb, zb, 0);
M1_r_fl_q6 = generate_approx_M_derivative(mr_fl, sigma_0, sigma_1, p_fl, (s_fl + d_fl), (h_fl - l_fl_0), -l_fl_1);
M1_r_fr_q6 = generate_approx_M_derivative(mr_fr, sigma_0, sigma_1, p_fr, (s_fr + d_fr), (h_fr - l_fr_0), -l_fr_1);
M1_r_rl_q6 = generate_approx_M_derivative(mr_rl, sigma_0, sigma_1, p_rl, (s_rl + d_rl), (h_rl - l_rl_0), -l_rl_1);
M1_r_rr_q6 = generate_approx_M_derivative(mr_rr, sigma_0, sigma_1, p_rr, (s_rr + d_rr), (h_rr - l_rr_0), -l_rr_1);
M1_q6 = M1_c_q6 + M1_r_fl_q6 + M1_r_fr_q6 + M1_r_rl_q6 + M1_r_rr_q6;
M1 = [M1_q6 zeros(6, 4); zeros(4, 10)];

% Derivative of Kinetic Energy with respect to Position
% (Approximated form)
% from "8/5/20 - 4"
dEcc_dsigma = +zb*mc*cos(sigma_0)*xc_1*rho_1 - zb*mc*sin(sigma_0)*xc_1*beta_1 - ...
    (-yb*sin(sigma_0) + xb*cos(sigma_0))*mc*xc_1*sigma_1 + ...
    zb*mc*sin(sigma_0)*yc_1*rho_1 + zb*mc*cos(sigma_0)*yc_1*beta_1 - ...
    (yb*cos(sigma_0) + xb*sin(sigma_0))*mc*yc_1*sigma_1;

dEcrv_fl = approx_wheel_kin_energy_derivatives(mr_fl, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_fl, s_fl, d_fl, h_fl, l_fl_0);
dEcrv_fr = approx_wheel_kin_energy_derivatives(mr_fr, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_fr, s_fr, d_fr, h_fr, l_fr_0);
dEcrv_rl = approx_wheel_kin_energy_derivatives(mr_rl, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_rl, s_rl, d_rl, h_rl, l_rl_0);
dEcrv_rr = approx_wheel_kin_energy_derivatives(mr_rr, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_rr, s_rr, d_rr, h_rr, l_rr_0);

dEcc_dq = zeros(10, 1);
dEcc_dq(6) = dEcc_dsigma + dEcrv_fl(1) + dEcrv_fr(1) + dEcrv_rl(1) + dEcrv_rr(1);
dEcc_dq(7) = dEcrv_fl(2);
dEcc_dq(8) = dEcrv_fr(2);
dEcc_dq(9) = dEcrv_rl(2);
dEcc_dq(10) = dEcrv_rr(2);

%% Gravitational contribution
% Car body
g = 9.806;
dVgc_dq_T = [0; 0; 1; (xb*sin(beta_0)*sin(rho_0) + yb*cos(rho_0) - zb*sin(rho_0)*cos(beta_0)); ...
    (-xb*cos(beta_0)*cos(rho_0) - zb*cos(rho_0)*sin(beta_0)); 0; 0; 0; 0; 0].*mc.*g;

% Wheels
% dVgr_dqr = grav_energy_deriv_wheel(p, s, d, h, l, rho, beta, sigma)
dVgr_fl_dqr = mr_fl*g*grav_energy_deriv_wheel(p_fl, s_fl, d_fl, h_fl, l_fl_0, rho_0, beta_0, sigma_0);
dVgr_fr_dqr = mr_fr*g*grav_energy_deriv_wheel(p_fr, s_fr, d_fr, h_fr, l_fr_0, rho_0, beta_0, sigma_0);
dVgr_rl_dqr = mr_rl*g*grav_energy_deriv_wheel(p_rl, s_rl, d_rl, h_rl, l_rl_0, rho_0, beta_0, sigma_0);
dVgr_rr_dqr = mr_rr*g*grav_energy_deriv_wheel(p_rr, s_rr, d_rr, h_rr, l_rr_0, rho_0, beta_0, sigma_0);
dVgr_dqr_sm = dVgr_fl_dqr + dVgr_fr_dqr + dVgr_rl_dqr + dVgr_rr_dqr;
dVgr_dq_T = [dVgr_dqr_sm(1:6); dVgr_fl_dqr(7); dVgr_fr_dqr(7); dVgr_rl_dqr(7); dVgr_rr_dqr(7)];

% total
dVg_dq_T = dVgc_dq_T + dVgr_dq_T;

%% Elastic contribution
% Suspension ("3/5/20 - 3")
dVkl_fl_qs = ks_fl*(l_fl_0 - l_fl_ind);
dVkl_fr_qs = ks_fr*(l_fr_0 - l_fr_ind);
dVkl_rl_qs = ks_rl*(l_rl_0 - l_rl_ind);
dVkl_rr_qs = ks_rr*(l_rr_0 - l_rr_ind);
dVkl_dq_T = [zeros(6, 1); dVkl_fl_qs; dVkl_fr_qs; dVkl_rl_qs; dVkl_rr_qs];

% Tyre/Wheel
% Calculating support coordinates r and speeds
r_fl_v = calc_r(zc_0, zc_1, zpn_fl_0, zpn_fl_1, p_fl, s_fl, d_fl, h_fl, l_fl_0, l_fl_1, rho_0, beta_0, rho_1, beta_1);
r_fr_v = calc_r(zc_0, zc_1, zpn_fr_0, zpn_fr_1, p_fr, s_fr, d_fr, h_fr, l_fr_0, l_fr_1, rho_0, beta_0, rho_1, beta_1);
r_rl_v = calc_r(zc_0, zc_1, zpn_rl_0, zpn_rl_1, p_rl, s_rl, d_rl, h_rl, l_rl_0, l_rl_1, rho_0, beta_0, rho_1, beta_1);
r_rr_v = calc_r(zc_0, zc_1, zpn_rr_0, zpn_rr_1, p_rr, s_rr, d_rr, h_rr, l_rr_0, l_rr_1, rho_0, beta_0, rho_1, beta_1);
r_fl_0 = r_fl_v(1); r_fl_1 = r_fl_v(2);
r_fr_0 = r_fr_v(1); r_fr_1 = r_fr_v(2);
r_rl_0 = r_rl_v(1); r_rl_1 = r_rl_v(2);
r_rr_0 = r_rr_v(1); r_rr_1 = r_rr_v(2);

% I segni non mi convincono molto; controllare.
dVkp_fl_dq = tyre_spring_potential_derivative(kp_fl, r_fl_0, r_fl_ind, p_fl, s_fl, d_fl, h_fl, l_fl_0, rho_0, beta_0, zc_0, zpn_fl_0);
dVkp_fr_dq = tyre_spring_potential_derivative(kp_fr, r_fr_0, r_fr_ind, p_fr, s_fr, d_fr, h_fr, l_fr_0, rho_0, beta_0, zc_0, zpn_fr_0);
dVkp_rl_dq = tyre_spring_potential_derivative(kp_rl, r_rl_0, r_rl_ind, p_rl, s_rl, d_rl, h_rl, l_rl_0, rho_0, beta_0, zc_0, zpn_rl_0);
dVkp_rr_dq = tyre_spring_potential_derivative(kp_rr, r_rr_0, r_rr_ind, p_rr, s_rr, d_rr, h_rr, l_rr_0, rho_0, beta_0, zc_0, zpn_rr_0);

% Assembling
dVkp_dq_T = [dVkp_fl_dq(1:6, 1) + dVkp_fr_dq(1:6, 1) + dVkp_rl_dq(1:6, 1) + dVkp_rr_dq(1:6, 1);
    dVkp_fl_dq(7, 1); dVkp_fr_dq(7, 1); dVkp_rl_dq(7, 1); dVkp_rr_dq(7, 1)];

dV_dq_T = dVg_dq_T + dVkl_dq_T + dVkp_dq_T;

%% Viscous damping
% Suspension ("3/5/20 - 7")
dDs_fl_qs = rs_fl*(l_fl_1);
dDs_fr_qs = rs_fr*(l_fr_1);
dDs_rl_qs = rs_rl*(l_rl_1);
dDs_rr_qs = rs_rr*(l_rr_1);
dDs_dq_T = [zeros(6, 1); dDs_fl_qs; dDs_fr_qs; dDs_rl_qs; dDs_rr_qs];

% Tyres ("8/5/20 -2")
% dDp_dq = tyre_damper_derivative(rp, r_0, r_1, p, s, d, h, l, rho, beta)
dDp_fl_dq = tyre_damper_derivative(rp_fl, r_fl_0, r_fl_1, p_fl, s_fl, d_fl, h_fl, l_fl_0, rho_0, beta_0);
dDp_fr_dq = tyre_damper_derivative(rp_fr, r_fr_0, r_fr_1, p_fr, s_fr, d_fr, h_fr, l_fr_0, rho_0, beta_0);
dDp_rl_dq = tyre_damper_derivative(rp_rl, r_rl_0, r_rl_1, p_rl, s_rl, d_rl, h_rl, l_rl_0, rho_0, beta_0);
dDp_rr_dq = tyre_damper_derivative(rp_rr, r_rr_0, r_rr_1, p_rr, s_rr, d_rr, h_rr, l_rr_0, rho_0, beta_0);

% Assembling
dDp_dq_T = [dDp_fl_dq(1:6, 1) + dDp_fr_dq(1:6, 1) + dDp_rl_dq(1:6, 1) + dDp_rr_dq(1:6, 1);
    dDp_fl_dq(7, 1); dDp_fr_dq(7, 1); dDp_rl_dq(7, 1); dDp_rr_dq(7, 1)];

dD_dq_T = dDs_dq_T + dDp_dq_T;

%% External forces
% Still unsure whether to put tyre moments. Wait until I think about it.

% Longitudinal and lateral forces acting onto tyre-road interface
% Q = reproject_tyre_forces(Fx, Fy, p, s, d, h, l, r, rho, beta, sigma)
Qp_fl = reproject_tyre_forces(Fx_fl, Fy_fl, p_fl, s_fl, d_fl, h_fl, l_fl_0, r_fl_0, rho_0, beta_0, sigma_0, zc_0, zpn_fl_0)';
Qp_fr = reproject_tyre_forces(Fx_fr, Fy_fr, p_fr, s_fr, d_fr, h_fr, l_fr_0, r_fr_0, rho_0, beta_0, sigma_0, zc_0, zpn_fr_0)';
Qp_rl = reproject_tyre_forces(Fx_rl, Fy_rl, p_rl, s_rl, d_rl, h_rl, l_rl_0, r_rl_0, rho_0, beta_0, sigma_0, zc_0, zpn_rl_0)';
Qp_rr = reproject_tyre_forces(Fx_rr, Fy_rr, p_rr, s_rr, d_rr, h_rr, l_rr_0, r_rr_0, rho_0, beta_0, sigma_0, zc_0, zpn_rr_0)';

Qp = [Qp_fl(1:6, 1) + Qp_fr(1:6, 1) + Qp_rl(1:6, 1) + Qp_rr(1:6, 1);
    Qp_fl(7, 1); Qp_fr(7, 1); Qp_rl(7, 1); Qp_rr(7, 1)];

% Aerodynamic forces (from "8/5/20 - 6");
% Drag: 
rho = 1.22; % air density
Cx = 0.3; % move outside 
S = 2;
Fd_x = -.5*rho*Cx*S*xc_1*abs(xc_1);
Fd_y = -.5*rho*Cx*S*yc_1*abs(yc_1);
Q_aer_d = [Fd_x; Fd_y; zeros(8, 1)];

% Lift
% TBD. (Formulas are there).
Q_aer_l = zeros(10, 1);

Q = Qp + Q_aer_d + Q_aer_l;
% Q = [Qp(1); zeros(9, 1)];
% disp(Q(1));
% Q(2:end) = -Q(2:end);

%% Assembling final equation
DU = -M1*q_1 + dEcc_dq - dV_dq_T - dD_dq_T + Q;
% DU = dEcc_dq + dV_dq_T - dD_dq_T + Q;
q_2 = M \ DU;

par_out.q_2 = q_2;
par_out.q_1 = q_1;
par_out.q_0 = q_0;
par_out.M = M;
par_out.dEcc_dq = dEcc_dq;
par_out.dV_dq_T = dV_dq_T;
par_out.dD_dq_T = dD_dq_T;
par_out.Q = Q;
Fz_r_fl = -kp_fl*(r_fl_0 - r_fl_ind) - rp_fl*(r_fl_1);
Fz_r_fr = -kp_fr*(r_fr_0 - r_fr_ind) - rp_fr*(r_fr_1);
Fz_r_rl = -kp_rl*(r_rl_0 - r_rl_ind) - rp_rl*(r_rl_1);
Fz_r_rr = -kp_rr*(r_rr_0 - r_rr_ind) - rp_rr*(r_rr_1);
par_out.F_wheels = [Fz_r_fl, Fz_r_fr, Fz_r_rl, Fz_r_rr];
par_out.r_wheels = [r_fl_0, r_fr_0, r_rl_0, r_rr_0];
par_out.rv_wheels = [r_fl_1, r_fr_1, r_rl_1, r_rr_1];


% q_2 = diag(diag(M)) \ DU;

if t > 200   || (l_fl_0 < 0)
%      keyboard
end

xd = [q_2; zeros(4, 1); y(1:14)];


% keyboard







% disp(t)























