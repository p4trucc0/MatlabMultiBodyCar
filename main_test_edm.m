clear all
close all
clc

% Vz = zeros(28, 1);
% edm_car([], Vz);
Vz = [zeros(14, 1); 0; 0; .3; 0; 0; 0; .5; .5; .5; .5; 0; 0; 0; 0];

use_ode45 = false;

if use_ode45
[t, y] = ode45(@edm_car,[0 5],Vz);
end
tic
[te, ye] = euler_integration(@edm_car, 0, 10, .0005, Vz);
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

Fw = zeros(size(ye, 1), 4);
for ii = 1:length(te)
    Fw(ii, :) = pp(ii).F_wheels;
end

figure; plot(te, ye(:, 21), 'r-'); grid on; title('FL suspension');
figure; plot(te, ye(:, 23), 'r-'); grid on; title('RL suspension');
figure; plot(te, ye(:, 15), 'r-'); grid on; title('Movement along X');
figure; plot(te, ye(:, 16), 'r-'); grid on; title('Movement along Y');
figure; plot(te, ye(:, 17), 'r-'); grid on; title('Movement along Z');

figure;
plot(te, ye(:, 18), 'r-');
grid on; hold on;
plot(te, ye(:, 19), 'b-');
plot(te, ye(:, 20), 'k-');
title('Angles');
legend('Roll \rho', 'Pitch \beta', 'Yaw \sigma');

figure;
plot(te, Fw(:, 1), 'r-');
grid on; hold on;
plot(te, Fw(:, 2), 'm-');
plot(te, Fw(:, 3), 'b-');
plot(te, Fw(:, 4), 'c-');
title('Wheel Forces');
legend('FL', 'FR', 'RL', 'RR');


sim_out = struct()
sim_out.t = [te(1):(1/30):te(end)]';
sim_out.xv = interp1(te, ye(:, 15:24), sim_out.t)

save('Gradini.mat', 'sim_out');













