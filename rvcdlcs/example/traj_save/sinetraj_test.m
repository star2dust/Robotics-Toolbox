close all
clear

ls = 1;
T = 16;
vs = 2*pi/T;
tf = 3*T;
qqr = [6,6,0;
    tf+6,6,vs*tf];

dt = 0.01;
tqr = 0:dt:tf;
[qr, dqr] = sinetraj(qqr(1,:),ls,vs,tqr);

showtraj(qr,dqr,tqr);

