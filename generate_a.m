function A = generate_a(r, b, s)

A = [cos(b)     0    -sin(b)*cos(r);
        0       1       sin(r);
     sin(b)     0     cos(b)*cos(r)];