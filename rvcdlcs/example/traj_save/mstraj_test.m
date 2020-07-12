close all
clear


qqr = [0,1,0;
    3,1,0;
    4,2,pi/3;
    4,4,pi/3;
    4,6,0];

dt = 0.01;
dqrmax = [2,2,1];
[qr, dqr, ~, tqr] = mstraj_(qqr,dqrmax,dt,2);


showtraj(qr,dqr,tqr);