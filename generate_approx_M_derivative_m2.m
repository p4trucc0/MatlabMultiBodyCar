function M1 = generate_approx_M_derivative_m2(Mp, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, xu_0, yu_0, zu_0, xu_1, yu_1, zu_1)
% SECOND METHOD (29/05/2020)
% M is the physical (6x6) mass matrix!

% Speed with cardan angles
cw = [rho_1; beta_1; sigma_1];

% Jacobian matric
Lc0 = generate_jacobian(rho_0, beta_0, sigma_0);

% omega car w r to car
Ac0 = generate_a(rho_0, beta_0, sigma_0);
Ac1 = generate_a1(rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1);
wcc = Ac0*cw;
Wcc = v2M(wcc);

Lc1 = Lc0 * Wcc; % derivative of Jacobian matrix

Xb0 = v2M([xu_0, yu_0, zu_0])';
Xb1 = v2M([xu_1, yu_1, zu_1])';

% part NOT accounting for the suspension.
L_lin_0 = [ones(3) Lc0*Xb0*Ac0; zeros(3) Ac0];
L_lin_1 = [zeros(3) (Lc1*Xb0*Ac0 + Lc0*Xb1*Ac0 + Lc0*Xb0*Ac1); ...
    zeros(3) Ac1];

% part for susp vertical movement
L_add_0 = [Lc0(:, 3); zeros(3, 1)];
L_add_1 = [Lc1(:, 3); zeros(3, 1)];

L0 = [L_lin_0 L_add_0];
L1 = [L_lin_1 L_add_1];

M1 = L1'*Mp*L0 + L0'*Mp*L1;