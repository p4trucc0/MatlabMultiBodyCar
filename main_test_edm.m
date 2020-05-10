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