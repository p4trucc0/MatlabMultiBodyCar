function dVkp_dq = tyre_spring_potential_derivative(kp, r, r_i, p, s, d, h, l, rho, beta, zc, zpn)
% From "8/5/20 - 1"

dVk_dr = kp*(r - r_i);

dr_dzc = 1 / (cos(beta)*cos(rho));
dr_dl = -1;

dr_dbeta = ((-(h - l)*sin(beta)*cos(rho) - p*cos(beta)*cos(rho))*(cos(beta)*cos(rho)) - ...
    (-sin(beta)*cos(rho))*(zc - zpn + (h - l)*cos(beta)*cos(rho) + (s + d)*sin(rho) - p*sin(beta)*cos(rho))) / ...
    ((cos(beta)*cos(rho))^2);

dr_drho = ((-(h - l)*cos(beta)*sin(rho) + (s + d)*cos(rho) + p*sin(beta)*sin(rho))*(cos(beta)*cos(rho)) - ...
    (-cos(beta)*sin(rho))*(zc - zpn + (h - l)*cos(beta)*cos(rho) + (s + d)*sin(rho) - p*sin(beta)*cos(rho))) / ...
    ((cos(beta)*cos(rho))^2);

dVkp_dq = [0; 0; dVk_dr*dr_dzc; ...
    dVk_dr*dr_drho;
    dVk_dr*dr_dbeta;
    0;
    dVk_dr*dr_dl];