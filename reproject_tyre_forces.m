function Q = reproject_tyre_forces(Fx, Fy, p, s, d, h, l, r, rho, beta, sigma)
% from "3/5/20 - 4", "3/5/20 - 5" and " - 6"
% This ignores derivatives of r with respect to other q.ties, however the
% original derivative of force w.r. to r is quite low, hence I'm neglecting
% this unless problems arise.

% Repòrojecting forces along absolute RS
Fv_s = [Fx; Fy; 0]; % expressed in rotating RS.
Ls0 = [cos(sigma) -sin(sigma) 0; sin(sigma) cos(sigma) 0; 0 0 1];
Fv_0 = Ls0*Fv_s;

dxp0_dxc = 1; 
dxp0_dyc = 0;
dxp0_dzc = 0;

dxp0_dsigma = -p*cos(beta)*sin(sigma) - p*cos(sigma)*sin(beta)*sin(rho) - ...
    (s + d)*cos(sigma)*cos(rho) + (h - l - r)*(-sin(sigma)*sin(beta) + ...
    cos(sigma)*sin(rho)*cos(beta));

dxp0_drho = -p*sin(sigma)*sin(beta)*cos(rho) + (s + d)*sin(sigma)*sin(rho) + ...
    (h - l - r)*sin(sigma)*cos(rho)*cos(beta);

dxp0_dbeta = -p*sin(sigma)*cos(beta)*sin(rho) + (h - l - r)*(cos(sigma)*cos(beta) - ...
    sin(sigma)*sin(rho)*sin(beta));

dxp0_dl = -(cos(sigma)*sin(beta) + sin(sigma)*sin(rho)*cos(beta));

dyp0_dxc = 0;
dyp0_dyc = 1;
dyp0_dzc = 0;

dyp0_dsigma = p*(cos(sigma)*cos(beta) - sin(sigma)*sin(rho)*sin(beta)) - ...
    (s + d)*sin(sigma)*cos(rho) + (h - l - r)*(sin(beta)*cos(sigma) + ...
    sin(sigma)*sin(rho)*cos(beta));

dyp0_drho = p*cos(sigma)*cos(rho)*sin(beta) - (s + d)*cos(sigma)*sin(rho) + ...
    (h - l - r)*(-cos(sigma)*cos(rho)*cos(beta));

dyp0_dbeta = p*(-sin(sigma)*sin(beta) + cos(sigma)*sin(rho)*cos(beta)) + ...
    (h - l - r)*(cos(beta)*sin(sigma) + cos(sigma)*sin(rho)*sin(beta));

dyp0_dl = -(sin(beta)*sin(sigma) - cos(sigma)*sin(rho)*cos(beta));

% Assembling the Lf matrix
Lf = [dxp0_dxc dxp0_dyc dxp0_dzc dxp0_drho dxp0_dbeta dxp0_dsigma dxp0_dl;
    dyp0_dxc dyp0_dyc dyp0_dzc dyp0_drho dyp0_dbeta dyp0_dsigma dyp0_dl;
    zeros(1, 7)];

Q = Fv_0'*Lf;