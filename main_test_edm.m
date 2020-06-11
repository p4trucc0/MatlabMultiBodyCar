clear all
close all
clc

% Vz = zeros(28, 1);
% edm_car([], Vz);
Vz = [zeros(14, 1); 0; 0; .3; 0; 0; 0; .5; .5; .5; .5; 0; 0; 0; 0];
Vz(1) = 20; % 50

xd_max = 100*ones(14, 1);

steppo = .0025;

use_ode45 = false;

if use_ode45
[t, y] = ode45(@edm_car,[0 5],Vz);
end
tic
[te, ye] = euler_integration(@edm_car, 0, 10, steppo, Vz, 0, xd_max);
toc
tic
pp = [];
ye1 = zeros(size(ye));
for ii = 1:size(ye, 1)
    [ye1(ii, :), par] = edm_car(te(ii), ye(ii, :)');
    pp = [pp; par];
end

toc

if use_ode45
figure;
plot(t, y(:, 17), 'r-'); grid on; hold on
plot(te, ye(:, 17), 'b-');
end

Fzw = zeros(size(ye, 1), 4);
Fxw = zeros(size(ye, 1), 4);
Fyw = zeros(size(ye, 1), 4);
F_aer_x = zeros(size(ye, 1), 1);
SA = zeros(size(ye, 1), 4);
SR = zeros(size(ye, 1), 4);
phiv = zeros(size(ye, 1), 4);
A_car = zeros(size(ye, 1), 3);
for ii = 1:length(te)
    A_car(ii, :) = pp(ii).A_car;
    Fzw(ii, :) = pp(ii).Fz_wheels;
    Fxw(ii, :) = pp(ii).Fx_t_wheels;
    Fyw(ii, :) = pp(ii).Fy_t_wheels;
    F_aer_x(ii) = pp(ii).F_aer_x;
    SA(ii, :) = pp(ii).slip_angles;
    SR(ii, :) = pp(ii).slip_rates;
    phiv(ii, :) = pp(ii).phiv;
end

figure; plot(te, ye(:, 21), 'r-'); grid on; title('FL suspension');
figure; plot(te, ye(:, 23), 'r-'); grid on; title('RL suspension');
figure; plot(te, ye(:, 15), 'r-'); grid on; title('Movement along X');
figure; plot(te, ye(:, 16), 'r-'); grid on; title('Movement along Y');
figure; plot(te, ye(:, 17), 'r-'); grid on; title('Movement along Z');

figure;
plot(te, ye(:, 11), 'r-');
grid on; hold on;
plot(te, ye(:, 12), 'm-');
plot(te, ye(:, 13), 'b-');
plot(te, ye(:, 14), 'c-');
title('Wheel Speeds [rad/s]');
legend('FL', 'FR', 'RL', 'RR');


figure;
plot(te, F_aer_x, 'r-'); grid on; title('Aerodynamic Drag Force along X');

figure;
plot(te, ye(:, 18), 'r-');
grid on; hold on;
plot(te, ye(:, 19), 'b-');
plot(te, ye(:, 20), 'k-');
title('Angles');
legend('Roll \rho', 'Pitch \beta', 'Yaw \sigma');

figure;
plot(te, Fzw(:, 1), 'r-');
grid on; hold on;
plot(te, Fzw(:, 2), 'm-');
plot(te, Fzw(:, 3), 'b-');
plot(te, Fzw(:, 4), 'c-');
title('Wheel Forces Z');
legend('FL', 'FR', 'RL', 'RR');

figure;
plot(te, A_car(:, 1), 'r-');
grid on; hold on;
plot(te, A_car(:, 2), 'b-');
plot(te, A_car(:, 3), 'k-');
title('Accelerations in car RS');
legend('X', 'Y', 'Z');

figure;
plot(te, phiv(:, 1), 'r-');
grid on; hold on;
plot(te, phiv(:, 2), 'm-');
plot(te, phiv(:, 3), 'b-');
plot(te, phiv(:, 4), 'c-');
title('Steered angle');
legend('FL', 'FR', 'RL', 'RR');

figure;
plot(te, Fxw(:, 1), 'r-');
grid on; hold on;
plot(te, Fxw(:, 2), 'm-');
plot(te, Fxw(:, 3), 'b-');
plot(te, Fxw(:, 4), 'c-');
title('Wheel Forces X');
legend('FL', 'FR', 'RL', 'RR');

figure;
plot(te, Fyw(:, 1), 'r-');
grid on; hold on;
plot(te, Fyw(:, 2), 'm-');
plot(te, Fyw(:, 3), 'b-');
plot(te, Fyw(:, 4), 'c-');
title('Wheel Forces Y');
legend('FL', 'FR', 'RL', 'RR');

figure;
plot(te, SA(:, 1), 'r-');
grid on; hold on;
plot(te, SA(:, 2), 'm-');
plot(te, SA(:, 3), 'b-');
plot(te, SA(:, 4), 'c-');
title('Slip angles [deg]');
legend('FL', 'FR', 'RL', 'RR');

figure;
plot(te, SR(:, 1), 'r-');
grid on; hold on;
plot(te, SR(:, 2), 'm-');
plot(te, SR(:, 3), 'b-');
plot(te, SR(:, 4), 'c-');
title('Slip Rates [%]');
legend('FL', 'FR', 'RL', 'RR');


sim_out = struct();
sim_out.t = [te(1):(1/30):te(end)]';
%          1------14  15-18 22 26  30  34  38 39------52  53-66 67-69
ye_out = [ye(:, 15:28) phiv SA SR Fzw Fxw Fyw ye(:, 1:14) ye1(:, 1:14)  A_car];
pre_smooth = true;
if pre_smooth
    ye_out = smoothdata(ye_out, 'movmean', round(.1/.0005));
end
sim_out.xv = interp1(te, ye_out, sim_out.t);

% Create a better output format.
sim_out.xc_0 = sim_out.xv(:, 1);
sim_out.yc_0 = sim_out.xv(:, 2);
sim_out.zc_0 = sim_out.xv(:, 3);
sim_out.rho_0 = sim_out.xv(:, 4);
sim_out.beta_0 = sim_out.xv(:, 5);
sim_out.sigma_0 = sim_out.xv(:, 6);
sim_out.l_fl_0 = sim_out.xv(:, 7);
sim_out.l_fr_0 = sim_out.xv(:, 8);
sim_out.l_rl_0 = sim_out.xv(:, 9);
sim_out.l_rr_0 = sim_out.xv(:, 10);
sim_out.th_fl_0 = sim_out.xv(:, 11);
sim_out.th_fr_0 = sim_out.xv(:, 12);
sim_out.th_rl_0 = sim_out.xv(:, 13);
sim_out.th_rr_0 = sim_out.xv(:, 14);
sim_out.xc_1 = sim_out.xv(:, 39);
sim_out.yc_1 = sim_out.xv(:, 40);
sim_out.zc_1 = sim_out.xv(:, 41);
sim_out.rho_1 = sim_out.xv(:, 42);
sim_out.beta_1 = sim_out.xv(:, 43);
sim_out.sigma_1 = sim_out.xv(:, 44);
sim_out.l_fl_1 = sim_out.xv(:, 45);
sim_out.l_fr_1 = sim_out.xv(:, 46);
sim_out.l_rl_1 = sim_out.xv(:, 47);
sim_out.l_rr_1 = sim_out.xv(:, 48);
sim_out.th_fl_1 = sim_out.xv(:, 49);
sim_out.th_fr_1 = sim_out.xv(:, 50);
sim_out.th_rl_1 = sim_out.xv(:, 51);
sim_out.th_rr_1 = sim_out.xv(:, 52);
sim_out.xc_2 = sim_out.xv(:, 53);
sim_out.yc_2 = sim_out.xv(:, 54);
sim_out.zc_2 = sim_out.xv(:, 55);
sim_out.rho_2 = sim_out.xv(:, 56);
sim_out.beta_2 = sim_out.xv(:, 57);
sim_out.sigma_2 = sim_out.xv(:, 58);
sim_out.l_fl_2 = sim_out.xv(:, 59);
sim_out.l_fr_2 = sim_out.xv(:, 60);
sim_out.l_rl_2 = sim_out.xv(:, 61);
sim_out.l_rr_2 = sim_out.xv(:, 62);
sim_out.th_fl_2 = sim_out.xv(:, 63);
sim_out.th_fr_2 = sim_out.xv(:, 64);
sim_out.th_rl_2 = sim_out.xv(:, 65);
sim_out.th_rr_2 = sim_out.xv(:, 66);
sim_out.phi_fl = sim_out.xv(:, 15);
sim_out.phi_fr = sim_out.xv(:, 16);
sim_out.phi_rl = sim_out.xv(:, 17);
sim_out.phi_rr = sim_out.xv(:, 18);
sim_out.sa_fl = sim_out.xv(:, 19);
sim_out.sa_fr = sim_out.xv(:, 20);
sim_out.sa_rl = sim_out.xv(:, 21);
sim_out.sa_rr = sim_out.xv(:, 22);
sim_out.sr_fl = sim_out.xv(:, 23);
sim_out.sr_fr = sim_out.xv(:, 24);
sim_out.sr_rl = sim_out.xv(:, 25);
sim_out.sr_rr = sim_out.xv(:, 26);
sim_out.Fz_fl = sim_out.xv(:, 27);
sim_out.Fz_fr = sim_out.xv(:, 28);
sim_out.Fz_rl = sim_out.xv(:, 29);
sim_out.Fz_rr = sim_out.xv(:, 30);
sim_out.Fx_fl = sim_out.xv(:, 31);
sim_out.Fx_fr = sim_out.xv(:, 32);
sim_out.Fx_rl = sim_out.xv(:, 33);
sim_out.Fx_rr = sim_out.xv(:, 34);
sim_out.Fy_fl = sim_out.xv(:, 35);
sim_out.Fy_fr = sim_out.xv(:, 36);
sim_out.Fy_rl = sim_out.xv(:, 37);
sim_out.Fy_rr = sim_out.xv(:, 38);
sim_out.Ax = sim_out.xv(:, 67);
sim_out.Ay = sim_out.xv(:, 68);
sim_out.Az = sim_out.xv(:, 69);

sim_out.Speed_ms = (sim_out.xc_1.^2 + sim_out.yc_1.^2 + sim_out.zc_1.^2).^.5;
sim_out.Speed_kmh = 3.6*sim_out.Speed_ms;

save('DeStiffing_Step_0025_DES000.mat', 'sim_out');













