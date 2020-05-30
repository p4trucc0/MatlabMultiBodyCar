function A = generate_a1(rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1)

A11 = -beta_1*sin(beta_0);
A12 = 0;
A13 = -beta_1*cos(beta_0)*cos(rho_0) + rho_1*sin(beta_0)*sin(rho_0);
A21 = 0;
A22 = 0;
A23 = rho_1*cos(rho_0);
A31 = beta_1*cos(beta_0);
A32 = 0;
A33 = -beta_1*sin(beta_0)*cos(rho_0) - rho_1*cos(beta_0)*sin(rho_0);

A = [A11 A12 A13; A21 A22 A23; A31 A32 A33];