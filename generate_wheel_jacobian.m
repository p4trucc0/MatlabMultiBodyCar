function L = generate_wheel_jacobian(rho, beta, sigma, p, s, d, h, l)
% distances from center of car Reference System
% "2/5/20 - 4"
xr_c = [p; s+d; h-l];
Lc0 = generate_jacobian(rho, beta, sigma);
Xr0c = v2M(xr_c)';
Athc = generate_a(rho, beta, sigma);

CM = Lc0 * Xr0c * Athc;

vl = Lc0(:, 3).*(-1);

L = [eye(3) CM vl];



