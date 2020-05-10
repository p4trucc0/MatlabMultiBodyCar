function dec_out = approx_wheel_kin_energy_derivatives(mr, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p, s, d, h, l)
% from "8/5/20 - 4"

dEcr_dsigma = +(h - l)*mr*cos(sigma_0)*xc_1*rho_1 - ...
    (h - l)*mr*sin(sigma_0)*xc_1*beta_1 - ...
    (p*cos(sigma_0) - (s + d)*sin(sigma_0))*mr*xc_1*sigma_1 + ...
    (h - l)*mr*sin(sigma_0)*yc_1*rho_1 + ...
    (h - l)*mr*cos(sigma_0)*yc_1*beta_1 - ...
    ((s + d)*sin(sigma_0) + p*sin(sigma_0));

dEcr_dl = -mr*sin(sigma_0)*xc_1*rho_1 - ...
    mr*cos(sigma_0)*xc_1*beta_1 + ...
    mr*cos(sigma_0)*yc_1*rho_1 - ...
    mr*sin(sigma_0)*yc_1*beta_1 + ...
    mr*(l - h)*(rho_1^2) + ...
    mr*(l - h)*(beta_1^2);

dec_out = [dEcr_dsigma; dEcr_dl];




