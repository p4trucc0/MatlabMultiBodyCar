function [Fx, Fy, Mz] = pacejka96(param, Fz, g, k, a)
% Pacejka 96 tyre model. Returns independently-calculated longitudinal and
% lateral forces, as well as self-aligning moment
% param: struct containing such fields as "a0", "b11", etc.
% Fz: vertical load in kN
% g: tyre IA in degrees
% k: longitudinal slip ratio in percentage
% a: sideslip angle in degrees


% Longitudinal Force
C = param.b0;
D = Fz * (param.b1*Fz + param.b2);
BCD = (param.b3*(Fz^2) + param.b4*Fz) * exp(-param.b5*Fz);
B = BCD / (C*D);
H = param.b9*Fz + param.b10;
E = (param.b6*(Fz^2) + param.b7*Fz + param.b8)*(1-param.b13*sign(k + H));
V = param.b11*Fz + param.b12;
Bx1 = B*(k + H);

Fx = D*sin(C*atan(Bx1 - E*(Bx1 - atan(Bx1)))) + V;

% Lateral Force
clear C D BCD B H E V Bx1
C = param.a0;
D = Fz*(param.a1*Fz + param.a2)*(1 - param.a15*(g^2));
BCD = param.a3*sin(atan(Fz/param.a4)*2)*(1 - param.a5*abs(g));
B = BCD / (C*D);
H = param.a8*Fz + param.a9 + param.a10*g;
E = (param.a6*Fz + param.a7)*(1 - (param.a16*g + param.a17)*sign(a + H));
V = param.a11*Fz + param.a12 + (param.a13*Fz + param.a14)*g*Fz;
Bx1 = B*(a + H);

Fy = D*sin(C*atan(Bx1 - E*(Bx1 - atan(Bx1)))) + V;

% Self-aligning moment
clear C D BCD B H E V Bx1
C = param.c0;
D = Fz*(param.c1*Fz + param.c2);
BCD = (param.c3*(Fz^2) + param.c4*Fz)*(1 - param.c6*abs(g))*exp(-param.c5*Fz);
B = BCD / (C*D);
H = param.c11*g + param.c12*Fz + param.c13;
E = (param.c7*(Fz^2) + param.c8*Fz + param.c9)*(1 - param.c10*abs(g));
V = (param.c14*(Fz^2) + param.c15*Fz)*g + param.c16*Fz + param.c17;
Bx1 = B*(a + H);
% Mz = D*sin(C*atan(Bx1 + E*atan(Bx1))) + V;
Mz = D*sin(C*atan(Bx1*(1-E) + E*atan(Bx1))) + V;

% keyboard
% Fy = 0;
% Mz = 0;

