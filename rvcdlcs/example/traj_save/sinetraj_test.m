close all
clear

ls = 4;
T = 16;
vs = 2*pi/T;
tf = 3*T;
qqr = [0,0;
    16,vs*tf];

dt = 0.01;
[qr, dqr, ddqr, t] = sinetraj(qqr(1,:),qqr(2,:),T,ls,dt);

showtraj(qr,dqr,t);

