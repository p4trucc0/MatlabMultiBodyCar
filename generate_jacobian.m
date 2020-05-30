function L = generate_jacobian(r, b, s)

l11 = cos(s)*cos(b) - sin(b)*sin(s)*sin(r);
l12 = -sin(s)*cos(r);
l13 = cos(s)*sin(b) + sin(s)*sin(r)*cos(b);
l21 = sin(s)*cos(b) + cos(s)*sin(r)*sin(b);
l22 = cos(s)*cos(r);
l23 = sin(b)*sin(s) - cos(s)*sin(r)*cos(b);
l31 = -sin(b)*cos(r);
l32 = sin(r);
l33 = cos(r)*cos(b);

L = [l11 l12 l13;
     l21 l22 l23;
     l31 l32 l33];

