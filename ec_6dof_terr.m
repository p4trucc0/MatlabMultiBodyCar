function [qd, par_out] = ec_6dof_terr(q, ptv)
% Patrucco, 2021
% 6-DOF model of a vehicle (plus 4 for wheel speeds and pos)
% includes variable height terrain
% in subsequent releases, both the terrain handle function and vehicle
% parameters should be supplied through additional arguments.

% q: 12 x 1 (6 free coordinates: first speeds, then positions): xc, yc, zc, rho_c, beta_c, sigma_c
% qd: derivation of q: 6 accelerations and 6 speeds.
% ptv: previous tyre (contact points) positions.
% 4 rows by 4 columns. Rows: x, y, z, l; columns: FL, FR, RL, RR

g = 9.806;

par_out = struct();

q_1 = q(1:10, 1);
q_0 = q(11:20, 1);

xc_1 = q(1);
yc_1 = q(2);
zc_1 = q(3);
rho_1 = q(4);
beta_1 = q(5);
sigma_1 = q(6);
theta_fl_1 = q(7);
theta_fr_1 = q(8);
theta_rl_1 = q(9);
theta_rr_1 = q(10);
xc_0 = q(11);
yc_0 = q(12);
zc_0 = q(13);
rho_0 = q(14);
beta_0 = q(15);
sigma_0 = q(16);
theta_fl_0 = q(17);
theta_fr_0 = q(18);
theta_rl_0 = q(19);
theta_rr_0 = q(20);


% Vehicle Parameters (these should go in by an additional parameter).
% car body center of gravity position.
mc = 1500; % kg.
Ixc = 300; %kg * m
Iyc = 2000; %kg * m
Izc = 2000; %kg * m

% Suspension geometry
p_f = 1.4; %m, distance of front axle from car RS
s_f = 0.7; %m, lateral distance front wheels
h_f = 0.8; %m, height of suspension mount point.
l_f_ind = 0.9; %m, undeformed suspension length
ks_f = 60000; % N/m, suspension spring
karb_f = 10000; % N/m, front anti-roll bar
k3_f = 0; %N/m, front "third element"
l3_f_ind = 0.9; % undeformed third element suspension length
rs_f = 10000; % N*s/m, damper

p_r = 1.4; %m, distance of rear axle from car RS
s_r = 0.7; %m, lateral distance front wheels
h_r = 0.8; %m, height of suspension mount point.
l_r_ind = 0.9; %m, undeformed suspension length
ks_r = 60000; % N/m, suspension spring
karb_r = 10000; % N/m, front anti-roll bar
k3_r = 0; %N/m, front "third element"
l3_r_ind = 0.9; % undeformed third element suspension length
rs_r = 10000; % N*s/m, damper

mr_f = 40; %kg, wheel + part of suspension + brake
mr_r = 20; 
md_f = 0; % differential mass
md_r = 100; % differential mass
Jr_f = .08; % radial inertia of front wheel
Jr_r = .08; % radial inertia of rear wheel
rr_f = 0.3; % rolling radius
rr_r = 0.3;

% Steering control parameter. This should be parametrized as well.
delta_fl = 0.0;
delta_fr = 0.0;
delta_rl = 0.0;
delta_rr = 0.0;

% Transform all geometric values for car positions
p_fl =  p_f; s_fl =  s_f; h_fl = h_f; mr_fl = mr_f; Jr_fl = Jr_f; rr_fl = rr_f;
p_fr =  p_f; s_fr = -s_f; h_fr = h_f; mr_fr = mr_f; Jr_fr = Jr_f; rr_fr = rr_f;
p_rl = -p_r; s_rl =  s_r; h_rl = h_r; mr_rl = mr_f; Jr_rl = Jr_r; rr_rl = rr_r;
p_rr = -p_r; s_rr = -s_r; h_rr = h_r; mr_rr = mr_f; Jr_rr = Jr_r; rr_rr = rr_r;
Xh_fl = [p_fl; s_fl; h_fl];
Xh_fr = [p_fr; s_fr; h_fr];
Xh_rl = [p_rl; s_rl; h_rl];
Xh_rr = [p_rr; s_rr; h_rr];

% Assigning supension values to each independent suspension.
ks_fl = ks_f; rs_fl = rs_f; l_fl_ind = l_f_ind; 
ks_fr = ks_f; rs_fr = rs_f; l_fr_ind = l_f_ind; 
ks_rl = ks_r; rs_rl = rs_r; l_rl_ind = l_r_ind; 
ks_rr = ks_r; rs_rr = rs_r; l_rr_ind = l_r_ind;

% Tyre parameters. Hard-coded, for now
tyre_param.a0 = 1.4660; tyre_param.a1 = -7.4; tyre_param.a2 = 999;
tyre_param.a3 = 2238; tyre_param.a4 = 8; tyre_param.a5 = 0.0150;
tyre_param.a6 = -0.2350; tyre_param.a7 = -0.3000; tyre_param.a8 = -0.0300;
tyre_param.a9 = -1.0000e-03; tyre_param.a10 = -0.1500; tyre_param.a111 = 0;
tyre_param.a112 = -0.2900; tyre_param.a12 = 17.8000; tyre_param.a13 = -2.4000;
tyre_param.b0 = 1.3600; tyre_param.b1 = -40; tyre_param.b2 = 1038;
tyre_param.b3 = 0.306; tyre_param.b4 = 100; tyre_param.b5 = 0.0800;
tyre_param.b6 = -0.0500; tyre_param.b7 = 0.0500; tyre_param.b8 = -0.0250;
tyre_param.b9 = 0.0; tyre_param.b10 = 0.0; tyre_param.c0 = 2.3150;
tyre_param.c1 = -4; tyre_param.c2 = -3; tyre_param.c3 = -1.6000; 
tyre_param.c4 = -6; tyre_param.c5 = 0; tyre_param.c6 = 0;
tyre_param.c7 = 0.0200; tyre_param.c8 = -0.5800; tyre_param.c9 = 0.1800;
tyre_param.c10 = 0.0430; tyre_param.c11 = 0.0480; tyre_param.c12 = -0.0035;
tyre_param.c13 = -0.1800; tyre_param.c14 = 0.1400; tyre_param.c15 = -1.0290;
tyre_param.c16 = 0.2700; tyre_param.c17 = -1.1000; tyre_param.a11 = 0;
tyre_param.a14 = 0; tyre_param.a15 = 0; tyre_param.a16 = 0;
tyre_param.a17 = 0; tyre_param.a18 = 0; tyre_param.a19 = 0;
tyre_param.a20 = 0; tyre_param.b11 = 0; tyre_param.b12 = 0;
tyre_param.b13 = 0; tyre_param.b14 = 0; tyre_param.b15 = 0;
tyre_param.b16 = 0; tyre_param.b17 = 0; tyre_param.b18 = 0;
tyre_param.b19 = 0; tyre_param.b20 = 0; tyre_param.c18 = 0;
tyre_param.c19 = 0; tyre_param.c20 = 0;
tyre_param.b9 = 0;
tyre_param.b10 = 0;

tyre_param_fl = tyre_param;
tyre_param_fr = tyre_param;
tyre_param_rl = tyre_param;
tyre_param_rr = tyre_param;

%% Get tyres positions
% This requires previous tyres positions. I should feed them somewhere in
% the code.
[p_a_fl, p_b_fl, p_c_fl, p_d_fl] = get_plane_coeffs(ptv(1, 1), ptv(2, 1), ptv(3, 1));
[p_a_fr, p_b_fr, p_c_fr, p_d_fr] = get_plane_coeffs(ptv(1, 2), ptv(2, 2), ptv(3, 2));
[p_a_rl, p_b_rl, p_c_rl, p_d_rl] = get_plane_coeffs(ptv(1, 3), ptv(2, 3), ptv(3, 3));
[p_a_rr, p_b_rr, p_c_rr, p_d_rr] = get_plane_coeffs(ptv(1, 4), ptv(2, 4), ptv(3, 4));
Lc0 = generate_rotation_matrix(rho_0, beta_0, sigma_0);
[B_fl, Bm_fl, dB_fl] = get_B_matrix(Lc0, p_a_fl, p_b_fl, p_c_fl);
[B_fr, Bm_fr, dB_fr] = get_B_matrix(Lc0, p_a_fr, p_b_fr, p_c_fr);
[B_rl, Bm_rl, dB_rl] = get_B_matrix(Lc0, p_a_rl, p_b_rl, p_c_rl);
[B_rr, Bm_rr, dB_rr] = get_B_matrix(Lc0, p_a_rr, p_b_rr, p_c_rr);
Xc = [xc_0; yc_0; zc_0];
X_fl_0 = get_X_ij(B_fl, Lc0, Xc, Xh_fl, p_d_fl);
X_fr_0 = get_X_ij(B_fr, Lc0, Xc, Xh_fr, p_d_fr);
X_rl_0 = get_X_ij(B_rl, Lc0, Xc, Xh_rl, p_d_rl);
X_rr_0 = get_X_ij(B_rr, Lc0, Xc, Xh_rr, p_d_rr);

%% 1 Kinetic contribution
% 1.1 - Vehicle body
% TODO: Find a better approximation for physical matrix.
Mc_phys = diag([mc mc mc Ixc Iyc Izc]);
m_tot = mc + mr_fl + mr_fr + mr_rl + mr_rr + md_f + md_r;
Mc_phys_approx1 = diag([m_tot m_tot mc Ixc Iyc Izc]);
L_mc = [eye(3), zeros(3); zeros(3), generate_a(rho_0, beta_0, sigma_0)];
Mc = L_mc'*Mc_phys*L_mc;
Mc_approx1 = L_mc'*Mc_phys_approx1*L_mc;

% 1.2 - Generic Tyre
% Calculate jacobian matrices
Jac_fl = get_tyre_jacobian(B_fl, Bm_fl, dB_fl, rho_0, beta_0, sigma_0, ...
    p_a_fl, p_b_fl, p_c_fl, p_d_fl, Lc0, Xh_fl, Xc);
Jac_fr = get_tyre_jacobian(B_fr, Bm_fr, dB_fr, rho_0, beta_0, sigma_0, ...
    p_a_fr, p_b_fr, p_c_fr, p_d_fr, Lc0, Xh_fr, Xc);
Jac_rl = get_tyre_jacobian(B_rl, Bm_rl, dB_rl, rho_0, beta_0, sigma_0, ...
    p_a_rl, p_b_rl, p_c_rl, p_d_rl, Lc0, Xh_rl, Xc);
Jac_rr = get_tyre_jacobian(B_rr, Bm_rr, dB_rr, rho_0, beta_0, sigma_0, ...
    p_a_rr, p_b_rr, p_c_rr, p_d_rr, Lc0, Xh_rr, Xc);
Mr_fl_phys = mr_fl*eye(3);
Mr_fr_phys = mr_fr*eye(3);
Mr_rl_phys = mr_rl*eye(3);
Mr_rr_phys = mr_rr*eye(3);
Mr_fl = Jac_fl(1:3,:)'*Mr_fl_phys*Jac_fl(1:3,:);
Mr_fr = Jac_fr(1:3,:)'*Mr_fr_phys*Jac_fr(1:3,:);
Mr_rl = Jac_rl(1:3,:)'*Mr_rl_phys*Jac_rl(1:3,:);
Mr_rr = Jac_rr(1:3,:)'*Mr_rr_phys*Jac_rr(1:3,:);

% 1.3 Differentials and transmission parts.
Jac_f = .5*Jac_fl(1:3,:) + .5*Jac_fr(1:3,:);
Jac_r = .5*Jac_rl(1:3,:) + .5*Jac_rr(1:3,:);
Md_f_phys = md_f*eye(3);
Md_r_phys = md_r*eye(3);
Md_f = Jac_f'*Md_f_phys*Jac_f;
Md_r = Jac_r'*Md_r_phys*Jac_r;

% 1.4 Sum up all components
M = Mc + Mr_fl + Mr_fr + Mr_rl + Mr_rr + Md_f + Md_r;

%% 2 External forces
% 2.1 Tyre forces
% Note that they are applied according to the body's reference system, i.e.
% they're NOT projected along the plane the wheel is found upon. Which
% would probably be more correct.
L_rc_fl = generate_rotation_matrix(0, 0, delta_fl);
L_rc_fr = generate_rotation_matrix(0, 0, delta_fr);
L_rc_rl = generate_rotation_matrix(0, 0, delta_rl);
L_rc_rr = generate_rotation_matrix(0, 0, delta_rr);

L_0r_fl = L_rc_fl'*Lc0';
L_0r_fr = L_rc_fr'*Lc0';
L_0r_rl = L_rc_rl'*Lc0';
L_0r_rr = L_rc_rr'*Lc0';

% Weight, due to wheel and differential, in the absolute space frame
Fz_rd_fl_0 = [0; 0; g*(mr_fl + md_f/2)];
Fz_rd_fr_0 = [0; 0; g*(mr_fr + md_f/2)];
Fz_rd_rl_0 = [0; 0; g*(mr_rl + md_r/2)];
Fz_rd_rr_0 = [0; 0; g*(mr_rr + md_r/2)];
% Same, but in the wheel space frame
Fz_rd_fl_r = L_0r_fl*Fz_rd_fl_0;
Fz_rd_fr_r = L_0r_fr*Fz_rd_fr_0;
Fz_rd_rl_r = L_0r_rl*Fz_rd_rl_0;
Fz_rd_rr_r = L_0r_rr*Fz_rd_rr_0;
% Force due to suspension compression
X_fl_1 = Jac_fl*q_1(1:6, 1); l_fl_0 = X_fl_0(4, 1); l_fl_1 = X_fl_1(4, 1);
X_fr_1 = Jac_fr*q_1(1:6, 1); l_fr_0 = X_fr_0(4, 1); l_fr_1 = X_fr_1(4, 1);
X_rl_1 = Jac_rl*q_1(1:6, 1); l_rl_0 = X_rl_0(4, 1); l_rl_1 = X_rl_1(4, 1);
X_rr_1 = Jac_rr*q_1(1:6, 1); l_rr_0 = X_rr_0(4, 1); l_rr_1 = X_rr_1(4, 1);
Fz_s_fl_r = [0; 0; -ks_fl*(l_fl_0 - l_fl_ind) - rs_fl*l_fl_1];
Fz_s_fr_r = [0; 0; -ks_fr*(l_fr_0 - l_fr_ind) - rs_fr*l_fr_1];
Fz_s_rl_r = [0; 0; -ks_rl*(l_rl_0 - l_rl_ind) - rs_rl*l_rl_1];
Fz_s_rr_r = [0; 0; -ks_rr*(l_rr_0 - l_rr_ind) - rs_rr*l_rr_1];
Fz_ar_fl_r = [0; 0; karb_f*(l_fr_0 - l_fl_0)];
Fz_ar_fr_r = [0; 0; karb_f*(l_fl_0 - l_fr_0)];
Fz_ar_rl_r = [0; 0; karb_r*(l_rr_0 - l_rl_0)];
Fz_ar_rr_r = [0; 0; karb_r*(l_rl_0 - l_rr_0)];
Fz_3_fl_r = [0; 0; k3_f*(l3_f_ind - ((l_fl_0 + l_fr_0)/2))];
Fz_3_fr_r = [0; 0; k3_f*(l3_f_ind - ((l_fl_0 + l_fr_0)/2))];
Fz_3_rl_r = [0; 0; k3_r*(l3_r_ind - ((l_rr_0 + l_rl_0)/2))];
Fz_3_rr_r = [0; 0; k3_r*(l3_r_ind - ((l_rr_0 + l_rl_0)/2))];
F_susp_fl_r = Fz_s_fl_r + Fz_ar_fl_r + Fz_3_fl_r;
F_susp_fr_r = Fz_s_fr_r + Fz_ar_fr_r + Fz_3_fr_r;
F_susp_rl_r = Fz_s_rl_r + Fz_ar_rl_r + Fz_3_rl_r;
F_susp_rr_r = Fz_s_rr_r + Fz_ar_rr_r + Fz_3_rr_r;

Fz_temp_fl_r = Fz_rd_fl_r + F_susp_fl_r;
Fz_temp_fr_r = Fz_rd_fr_r + F_susp_fr_r;
Fz_temp_rl_r = Fz_rd_rl_r + F_susp_rl_r;
Fz_temp_rr_r = Fz_rd_rr_r + F_susp_rr_r;

% Speed of the contact point in the wheel reference frame
Lc_1 = generate_rotmatrix_dtime(rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1);
L_0r_fl_1 = L_rc_fl'*Lc_1;
L_0r_fr_1 = L_rc_fr'*Lc_1;
L_0r_rl_1 = L_rc_rl'*Lc_1;
L_0r_rr_1 = L_rc_rr'*Lc_1;
X_fl_1_r = L_0r_fl_1*X_fl_0(1:3, :) + L_0r_fl*X_fl_1(1:3, :);
X_fr_1_r = L_0r_fr_1*X_fr_0(1:3, :) + L_0r_fr*X_fr_1(1:3, :);
X_rl_1_r = L_0r_rl_1*X_rl_0(1:3, :) + L_0r_rl*X_rl_1(1:3, :);
X_rr_1_r = L_0r_rr_1*X_rr_0(1:3, :) + L_0r_rr*X_rr_1(1:3, :);

keyboard

%% SUBFUNCTIONS
    
    % get plane coefficients
    function [p_a, p_b, p_c, p_d] = get_plane_coeffs(x, y, z)
        % W.I.P. (It will depend on terrain and position)
        p_a = 0;
        p_b = 0;
        p_c = 1.0;
        p_d = 0.0;
    end

    % get B, Bm and dB matrices and scalar. MAKE SURE THIS IS OK
    function [B, Bm, dB] = get_B_matrix(Lc, p_a, p_b, p_c)
        if 0 % for debug purposes
            A = [1, 0, 0, Lc(1, 3);
                0, 1, 0, Lc(2, 3);
                0, 0, 1, Lc(3, 3);
                p_a, p_b, p_c, 0];
        end
        dB = (p_a*Lc(1, 3) + p_b*Lc(2, 3) + p_c*Lc(3, 3));
        Bm = [(p_b*Lc(2, 3) + p_c*Lc(3, 3)), -p_b*Lc(1, 3), -p_c*Lc(1, 3), Lc(1, 3);
            -p_a*Lc(2, 3), (p_a*Lc(1, 3) + p_c*Lc(3, 3)), -p_c*Lc(2, 3), Lc(2, 3);
            -p_a*Lc(3, 3), -p_b*Lc(3, 3), (p_a*Lc(1, 3) + p_b*Lc(2, 3)), Lc(3, 3);
            p_a, p_b, p_c, -1];
        B = (1/dB)*Bm;
        % keyboard
    end

    % der_by: 1: rho, 2: beta, 3: sigma.
    function B1 = get_B_derivative(Bm, dB, r, b, s, p_a, p_b, p_c, der_by)
        Lc1 = generate_rotmatrix_drvd(r, b, s, der_by);
        Lc13_1 = Lc1(1, 3);
        Lc23_1 = Lc1(2, 3);
        Lc33_1 = Lc1(3, 3);
        dB1 = (p_a*Lc1(1, 3) + p_b*Lc1(2, 3) + p_c*Lc1(3, 3));
        Bm1 = [(p_b*Lc1(2, 3) + p_c*Lc1(3, 3)), -p_b*Lc1(1, 3), -p_c*Lc1(1, 3), Lc1(1, 3);
            -p_a*Lc1(2, 3), (p_a*Lc1(1, 3) + p_c*Lc1(3, 3)), -p_c*Lc1(2, 3), Lc1(2, 3);
            -p_a*Lc1(3, 3), -p_b*Lc1(3, 3), (p_a*Lc1(1, 3) + p_b*Lc1(2, 3)), Lc1(3, 3);
            0, 0, 0, 0];
        B1 = (dB*Bm1 - dB1*Bm)./(dB^2);
    end

    function J_ij = get_tyre_jacobian(B, Bm, dB, r, b, s, p_a, p_b, p_c, p_d, Lc, Xh_ij, Xc)
        v4 = [(Lc*Xh_ij + Xc); p_d];
%         dXr_ij_dxc = B(1:3, :)*[1 0 0 0]';
%         dXr_ij_dyc = B(1:3, :)*[0 1 0 0]';
%         dXr_ij_dzc = B(1:3, :)*[0 0 1 0]';
        dXr_ij_dxc = B(:, :)*[1 0 0 0]';
        dXr_ij_dyc = B(:, :)*[0 1 0 0]';
        dXr_ij_dzc = B(:, :)*[0 0 1 0]';
        dB_ij_drho = get_B_derivative(Bm, dB, r, b, s, p_a, p_b, p_c, 1);
        dv4_drho = [generate_rotmatrix_drvd(r, b, s, 1)*Xh_ij; 0];
%         dXr_ij_drho = dB_ij_drho(1:3, :)*v4 + B(1:3, :)*dv4_drho;
        dXr_ij_drho = dB_ij_drho(:, :)*v4 + B(:, :)*dv4_drho;
        dB_ij_dbeta = get_B_derivative(Bm, dB, r, b, s, p_a, p_b, p_c, 2);
        dv4_dbeta = [generate_rotmatrix_drvd(r, b, s, 2)*Xh_ij; 0];
%         dXr_ij_dbeta = dB_ij_dbeta(1:3, :)*v4 + B(1:3, :)*dv4_dbeta;
        dXr_ij_dbeta = dB_ij_dbeta(:, :)*v4 + B(:, :)*dv4_dbeta;
        dB_ij_dsigma = get_B_derivative(Bm, dB, r, b, s, p_a, p_b, p_c, 3);
        dv4_dsigma = [generate_rotmatrix_drvd(r, b, s, 3)*Xh_ij; 0];
%         dXr_ij_dsigma = dB_ij_dsigma(1:3, :)*v4 + B(1:3, :)*dv4_dsigma;
        dXr_ij_dsigma = dB_ij_dsigma(:, :)*v4 + B(:, :)*dv4_dsigma;
        J_ij = [dXr_ij_dxc, dXr_ij_dyc, dXr_ij_dzc, dXr_ij_drho, ...
            dXr_ij_dbeta, dXr_ij_dsigma];
    end

    % TODO: This is inefficient. Apply omega rules instead.
%     function X_ij_1 = get_X_ij_1(B, Bm, dB, r, b, s, r1, b1, s1, p_a, p_b, p_c, p_d, Lc, Xh_ij, Xc)
%         v4_0 = [(Lc*Xh_ij + Xc); p_d];
%         dv4_drho = [generate_rotmatrix_drvd(r, b, s, 1)*Xh_ij; 0];
%         dv4_dbeta = [generate_rotmatrix_drvd(r, b, s, 2)*Xh_ij; 0];
%         dv4_dsigma = [generate_rotmatrix_drvd(r, b, s, 3)*Xh_ij; 0];
%         v4_1 = r1*dv4_drho + b1*dv4_dbeta + s1*dv4_dsigma;
%         X_ij_1 = B*v4_1 + get_B_derivative
%     end

    function X_ij = get_X_ij(B_ij, Lc, Xc, Xh_ij, p_d_ij)
        X_ij = B_ij*([Lc*Xh_ij + Xc; p_d_ij]);
    end



end