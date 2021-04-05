function Lio = generate_rotmatrix_dtime(r0, b0, s0, r1, b1, s1)

dLdr = generate_rotmatrix_drvd(r0, b0, s0, 1);
dLdb = generate_rotmatrix_drvd(r0, b0, s0, 2);
dLds = generate_rotmatrix_drvd(r0, b0, s0, 3);

Lio = r1*dLdr + b1*dLdb + s1*dLds;

end