function [qd, par_out] = ec_6dof_terr(q)
% Patrucco, 2021
% 6-DOF model of a vehicle (plus 4 for wheel speeds and pos)
% includes variable height terrain
% in subsequent releases, both the terrain handle function and vehicle
% parameters should be supplied through additional arguments.

% q: 12 x 1 (6 free coordinates: first speeds, then positions): xc, yc, zc, rho_c, beta_c, sigma_c
% qd: derivation of q: 6 accelerations and 6 speeds.


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

% Transform all geometric values for car positions
p_fl =  p_f; s_fl =  s_f; h_fl = h_f; mr_fl = mr_f; Jr_fl = Jr_f; rr_fl = rr_f;
p_fr =  p_f; s_fr = -s_f; h_fr = h_f; mr_fr = mr_f; Jr_fr = Jr_f; rr_fr = rr_f;
p_rl = -p_r; s_rl =  s_r; h_rl = h_r; mr_rl = mr_f; Jr_rl = Jr_r; rr_rl = rr_r;
p_rr = -p_r; s_rr = -s_r; h_rr = h_r; mr_rr = mr_f; Jr_rr = Jr_r; rr_rr = rr_r;

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








