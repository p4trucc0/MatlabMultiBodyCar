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
[te, ye] = euler_integration(@edm_car, 0, 5, .0005, Vz);
toc

if use_ode45
figure;
plot(t, y(:, 17), 'r-'); grid on; hold on
plot(te, ye(:, 17), 'b-');
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