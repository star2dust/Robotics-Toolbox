% SCARA robot kinematics and dynmaics calculation using my function
% reference book: Richard M. Murray-A Mathematical Introduction to Robotic Manipulation-CRC Press (1994). 
close all
clear

%% parameters
% link length
syms l1 l2 l3 real
% link center of mass
syms r1 r2 r3 real
% joint angle
syms th1 th2 th3 th4 real
% joint velocity
syms dth1 dth2 dth3 dth4 real
% link mass
syms m1 m2 m3 m4 real
% link inertia
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 Ix3 Iy3 Iz3 Ix4 Iy4 Iz4 real
% gravity and dampling coefficient
syms g b real
%% reference config
g_st0 = transl(0,l1+l2,0);
% choose q according to type of joints
q1 = [0,0,0]';
q2 = [0,l1,0]';
q3 = [0,l1+l2,0]';
w1 = [0,0,1]';
w2 = w1; w3 = w1; v4 = w1;
% joint twists
xi1 = [-skew(w1)*q1;w1];
xi2 = [-skew(w2)*q2;w2];
xi3 = [-skew(w3)*q3;w3];
xi4 = [v4;zeros(3,1)];
% reference config for each joint
g_sl0{1} = transl(0,r1,l3);
g_sl0{2} = transl(0,l1+r2,l3);
g_sl0{3} = transl(0,l1+l2,r3);
g_sl0{4} = transl(0,l1+l2,r3);
% make a matrix
xi = {xi1,xi2,xi3,xi4};
th = [th1,th2,th3,th4];
dth = [dth1,dth2,dth3,dth4];
m = [m1,m2,m3,m4];
I = {[Ix1 Iy1 Iz1],[Ix2 Iy2 Iz2],[Ix3 Iy3 Iz3],[Ix4,Iy4,Iz4]};
%% robot dynamics
[M,C,N] = dymat(m,I,xi,th,dth,g_sl0,{g,b});
% test Mr here by simplify(M-M_t) (reference from p199)
a_t = Iz1+r1^2*m1+l1^2*m2+l1^2*m3+l1^2*m4;
b_t = Iz2+Iz3+Iz4+l2^2*m3+l2^2*m4+m2*r2^2;
c_t = l1*l2*m3+l1*l2*m4+l1*m2*r2;
d_t = Iz3+Iz4;
M_t = [a_t+b_t+2*c_t*cos(th2),b_t+c_t*cos(th2),d_t,0;
    b_t+c_t*cos(th2),b_t,d_t,0;
    d_t,d_t,d_t,0;
    0,0,0,m4];
% test C here by simplify(C-C_t) (reference from p199)
C_t = [-c_t*sin(th2)*dth2,-c_t*sin(th2)*(dth1+dth2),0,0;
    c_t*sin(th2)*dth1,0,0,0;
    0,0,0,0;
    0,0,0,0];
% test N here by simplify(N-N_t) (reference from p200)
N_t = [0,0,0,m4*g]'+b*[dth1,dth2,dth3,dth4]';
% test validity
simplify(M-M_t)
simplify(C-C_t)
simplify(N-N_t)