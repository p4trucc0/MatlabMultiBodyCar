clear all
close all
clc

%% Patrucco, 22/05/2020
% Test funzione di calcolo forze e momento secondo Pacejka 96.

BaseTyreFile = 'tyre1.txt';

tyre_param = parse_tyre_params(BaseTyreFile);

tyre_param.b9 = 0;
tyre_param.b10 = 0;

Fz = 4000;

srv = linspace(-200, 200, 1000);
fxv = zeros(size(srv));
for ii = 1:length(srv)
    sr = srv(ii);
    [Fx, Fy, Mz] = pacejka96(tyre_param, Fz/1000, 0.0, sr, 0.0);
    fxv(ii) = Fx;
end

% [Fx, Fy, Mz] = pacejka96(tyre_param, 4.0, 0.0, 0.0, 0.0);

figure;
plot(srv, fxv./Fz, 'r-');
grid on;
xlabel('Slip Ratio [%]');
ylabel('Longitudinal Force [N/N]');



sav = linspace(-20, 20, 1000);
fyv = zeros(size(sav));
for ii = 1:length(sav)
    sa = sav(ii);
    [Fx, Fy, Mz] = pacejka96(tyre_param, Fz/1000, 0.0, 0.0, sa);
    fyv(ii) = Fy;
end

% [Fx, Fy, Mz] = pacejka96(tyre_param, 4.0, 0.0, 0.0, 0.0);

figure;
plot(sav, fyv./Fz, 'r-');
grid on;
xlabel('Slip Angle [°]');
ylabel('Lateral Force [N/N]');


sav = linspace(-20, 20, 1000);
mzv = zeros(size(sav));
for ii = 1:length(sav)
    sa = sav(ii);
    [Fx, Fy, Mz] = pacejka96(tyre_param, Fz/1000, 0.0, 0.0, sa);
    mzv(ii) = Mz;
end

% [Fx, Fy, Mz] = pacejka96(tyre_param, 4.0, 0.0, 0.0, 0.0);

figure;
plot(sav, mzv, 'r-');
grid on;
xlabel('Slip Angle [°]');
ylabel('Self-aligning moment [Nm]');

% For semi-random parameters, it works quite decently. 
