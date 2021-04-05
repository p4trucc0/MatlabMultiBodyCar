clear all
close all
clc

tyre_param = parse_tyre_params('pneumatico_piu_stradale.txt');
[Fx, Fy, Mz] = pacejka96(tyre_param, 5.0, 0.0, 10.0, 20.0);

Fz = 4000;
phi = 0.0;
p = 1.4;
s = 0.6;
d = 0.1;
h = 0.8;
l0 = 0.6;
l1 = 0.0;
rr = 0.3;
xc_1 = 20.0;
yc_1 = 10.0;
zc_1 = 0.0;
rho_0 = 0.0;
beta_0 = 0.0;
sigma_0 = 0.0;
rho_1 = 0.0;
beta_1 = 0.0;
sigma_1 = 0.0;
omega = 70.0;
symmetrize = 1;

tic
for ii = 1:100000
[Fx, Fy, Mz, SR, SA] = apply_pacejka(Fz, phi, p, s, d, h, l0, l1, rr, xc_1, yc_1, zc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, omega, tyre_param, symmetrize);
end
toc

disp(['Fx = ', num2str(Fx)]);
disp(['Fy = ', num2str(Fy)]);
disp(['Mz = ', num2str(Mz)]);
disp(['SA = ', num2str(SA)]);
disp(['SR = ', num2str(SR)]);