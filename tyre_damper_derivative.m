function dDp_dq = tyre_damper_derivative(rp, r_0, r_1, p, s, d, h, l, rho, beta)
% From "8/5/20 - 1"

dDp_dr = rp*r_1;

dr_dzc = 1/(cos(beta)*cos(rho));
dr_dl = -1;

dr_dbeta = (-(h - l - r_0)*sin(beta)*cos(rho) - p*cos(beta)*cos(rho)) / (cos(beta)*cos(rho));

dr_drho = (-(h - l - r_0)*cos(beta)*sin(rho) + (s + d)*cos(rho) + ...
    p*sin(beta)*sin(rho)) / (cos(beta)*cos(rho));

dDp_dq = [0; 0; dDp_dr*dr_dzc; ...
    dDp_dr*dr_drho; ...
    dDp_dr*dr_dbeta;
    0;
    dDp_dr*dr_dl];