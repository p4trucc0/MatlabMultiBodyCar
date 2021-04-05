clear all
close all
clc

%% Patrucco, 05/04/2021
% Test ec_6dof_terr function

q0 = zeros(20, 1);
% Introduce some angles
q0(14, 1) = 0.1;
q0(15, 1) = 0.05;
q0(16, 1) = 2.0;
q1 = ec_6dof_terr(q0, zeros(4, 4));

