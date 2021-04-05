function m = generate_rotmatrix_drvd(r, b, s, derived_by)
% Patrucco, 2021
% Generate derivated form of the rotation matrix (euler angles, s-r-b
% sequence). "derived_by" equals 1 for r, 2 for b, 3 for s

% Lc11 = cos(s)*cos(b) - sin(b)*sin(s)*sin(r)
dLc11_dr = -sin(b)*sin(s)*cos(r);
dLc11_db = -cos(s)*sin(b) - cos(b)*sin(s)*sin(r);
dLc11_ds = -sin(s)*cos(b) - sin(b)*cos(s)*sin(r);

% Lc12 = -sin(s)*cos(r)
dLc12_dr = sin(s)*sin(r);
dLc12_db = 0;
dLc12_ds = -cos(s)*cos(r);

% Lc13 = cos(s)*sin(b) + sin(s)*sin(r)*cos(b)
dLc13_dr = sin(s)*cos(r)*cos(b);
dLc13_db = cos(s)*cos(b) - sin(s)*sin(r)*sin(b);
dLc13_ds = -sin(s)*sin(b) + cos(s)*sin(r)*cos(b);

% Lc21 = sin(s)*cos(b) + cos(s)*sin(r)*sin(b)
dLc21_dr = cos(s)*cos(r)*sin(b);
dLc21_db = -sin(s)*sin(b) + cos(s)*sin(r)*cos(b);
dLc21_ds = cos(s)*cos(b) - sin(s)*sin(r)*sin(b);

% Lc22 = cos(s)*cos(r)
dLc22_dr = -cos(s)*sin(r);
dLc22_db = 0;
dLc22_ds = -sin(s)*cos(r);

% Lc23 = sin(b)*sin(s) - cos(s)*sin(r)*cos(b)
dLc23_dr = -cos(s)*cos(r)*cos(b);
dLc23_db = cos(b)*sin(s) + cos(s)*sin(r)*sin(b);
dLc23_ds = sin(b)*cos(s) + sin(s)*sin(r)*cos(b);

% Lc31 = -sin(b)*cos(r)
dLc31_dr = sin(b)*sin(r);
dLc31_db = -cos(b)*cos(r);
dLc31_ds = 0;

% Lc32 = sin(r)
dLc32_dr = cos(r);
dLc32_db = 0;
dLc32_ds = 0;

% Lc33 = cos(r)*cos(b)
dLc33_dr = -sin(r)*cos(b);
dLc33_db = -cos(r)*sin(b);
dLc33_ds = 0;

switch(derived_by)
    case 1
        m = [dLc11_dr, dLc12_dr, dLc13_dr;
            dLc21_dr, dLc22_dr, dLc23_dr;
            dLc31_dr, dLc32_dr, dLc33_dr];
    case 2
        m = [dLc11_db, dLc12_db, dLc13_db;
            dLc21_db, dLc22_db, dLc23_db;
            dLc31_db, dLc32_db, dLc33_db];
    case 3
        m = [dLc11_ds, dLc12_ds, dLc13_ds;
            dLc21_ds, dLc22_ds, dLc23_ds;
            dLc31_ds, dLc32_ds, dLc33_ds];
    otherwise
        m = zeros(3);
        warning('generate_rotmatrix_drvd: Invalid option for derivation');
end