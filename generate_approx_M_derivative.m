function M1 = generate_approx_M_derivative(m, sigma_0, sigma_1, xu_0, yu_0, zu_0, zu_1)
% from "8/5/20 - 5"

M1_11 = zeros(3, 3);
M1_12_11 = (+zu_1*m*sin(sigma_0) + zu_0*sigma_1*m*cos(sigma_0));
M1_12_12 = (+zu_1*m*cos(sigma_0) - zu_0*sigma_1*m*sin(sigma_0));
M1_12_13 = -(-yu_0*sin(sigma_0) + xu_0*cos(sigma_0))*m*sigma_1;
M1_12_21 = -(zu_1*m*cos(sigma_0) - zu_0*sigma_1*m*sin(sigma_0));
M1_12_22 = (zu_1*m*sin(sigma_0) + zu_0*sigma_1*m*cos(sigma_0));
M1_12_23 = -(yu_0*cos(sigma_0) + xu_0*sin(sigma_0))*m*sigma_1;
M1_12_3 = zeros(1, 3);
M1_12 = [M1_12_11 M1_12_12 M1_12_13;
         M1_12_21 M1_12_22 M1_12_23;
         M1_12_3];
M1_21 = M1_12'; % symmetric.
M1_22_11 = 2*m*zu_0*zu_1;
M1_22_12 = 0;
M1_22_13 = 0;
M1_22_21 = 0;
M1_22_22 = 2*m*zu_0*zu_1;
M1_22_23 = -m*yu_0*zu_1;
M1_22_31 = 0;
M1_22_32 = M1_22_23;
M1_22_33 = 0;
M1_22 = [M1_22_11 M1_22_12 M1_22_13;
         M1_22_21 M1_22_22 M1_22_23;
         M1_22_31 M1_22_32 M1_22_33];
M1 = [M1_11 M1_12;
      M1_21 M1_22];