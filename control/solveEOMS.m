% Destiny Fawley
% 11/21/2019

load('eom.mat');
syms PHI PHIDOT PSI PSIDOT Z ZDOT
x = [PHI; PHIDOT; PSI; PSIDOT; Z; ZDOT]
xe = [0.000000000 0.000000000 0.00000000 0.00000000 0.000000000 0.0000000000001];
theta = 0;
A1 = double(subs(jacobian(accel(3),x),x,xe'))
arot = double(subs(angAccel(1:2),x,xe'))