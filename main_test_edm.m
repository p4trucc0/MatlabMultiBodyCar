clear all
close all
clc

% Vz = zeros(28, 1);
% edm_car([], Vz);
Vz = [zeros(14, 1); 0; 0; .3; 0; 0; 0; .5; .5; .5; .5; 0; 0; 0; 0];
Vz(1) = 50; % 50


use_ode45 = false;

if use_ode45
[t, y] = ode45(@edm_car,[0 5],Vz);
end
tic
[te, ye] = euler_integration(@edm_car, 0, 10, .0005, Vz);
toc
tic
pp = [];
for ii = 1:size(ye, 1)
    [~, par] = edm_car(te(ii), ye(ii, :)');
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
for ii = 1:length(te)
    Fzw(ii, :) = pp(ii).Fz_wheels;
    Fxw(ii, :) = pp(ii).Fx_t_wheels;
    Fyw(ii, :) = pp(ii).Fy_t_wheels;
    F_aer_x(ii) = pp(ii).F_aer_x;
    SA(ii, :) = pp(ii).slip_angles;
    SR(ii, :) = pp(ii).slip_rates;
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


sim_out = struct()
sim_out.t = [te(1):(1/30):te(end)]';
sim_out.xv = interp1(te, ye(:, 15:24), sim_out.t)

save('Gradini.mat', 'sim_out');













