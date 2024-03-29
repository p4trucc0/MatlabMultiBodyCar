function dVgr_dqr = grav_energy_deriv_wheel(p, s, d, h, l, rho_0, beta_0, sigma_0)
% from "3/5/20 - 2"
dVgr_dqr = [0; 0; 1; ...
    (-p*cos(beta_0)*cos(rho_0) - (h - l)*cos(rho_0)*sin(beta_0)); ...
    (p*sin(beta_0)*sin(rho_0) + (s + d)*cos(rho_0) - sin(rho_0)*cos(beta_0)*(h - l)); ...
    0; ...
    -cos(rho_0)*cos(beta_0)];
    