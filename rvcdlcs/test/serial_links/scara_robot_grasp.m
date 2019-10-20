% Two SCARA fingers grasp an object calculation
% Reference book: Richard M. Murray-A Mathematical Introduction to Robotic Manipulation-CRC Press (1994).
close all
clear

%% soft-finger grasp => get G & B (page 241)
syms r real
% r = 1; % cuboid: 2r*r*r
g_oc{1} = transl(0,-r,0)*floor(trotz(-pi/2))*floor(troty(-pi/2));
g_oc{2} = transl(0,r,0)*floor(trotx(pi/2));
ex = [1,0,0,0,0,0]';
ey = [0,1,0,0,0,0]';
ez = [0,0,1,0,0,0]';
oz = [0,0,0,0,0,1]';
B_c = [ex,ey,ez,oz];
G = [Adg(invg(g_oc{1}))'*B_c, Adg(invg(g_oc{2}))'*B_c];
%% two scara fingers grasping a box => get J (page 261)
syms l1 l2 l3 l4 th1 th2 th3 th4 real
% SCARA forward kinematics (page 108)
% choose q according to type of joints (frame see Fig. 3.3)
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
% make a matrix
l = [l1,l2,l3,l4];
xi = {xi1,xi2,xi3,xi4};
th = [th1,th2,th3,th4];
n = length(th);
% Jacobian for a SCARA robot (page 139)
J_sf_s = cell(1,2);
for j=1:2
    J_sf_s{j} = sym(zeros(6,4)); % for symbol calculation
    J_sf_s{j}(:,1) = xi{1};
    for i=2:n
        J_sf_s{j}(:,i) = Adg(gtr(xi,th,0,i))*xi{i};
    end
end
% Adjoint tranformation associated with g_sf
syms a b real
% a = 0.5; % distance between P and O
% b = 1.5; % distance between P and S
g_po = transl([0,0,a]);
Ad_g_po = Adg(g_po);
g_sp{1} = transl([0,b,0]);
g_sp{2} = transl([0,-b,0]);
for i=1:2
    g_sc{i} = g_sp{i}*g_po*g_oc{i};
    Ad_inv_g_sc{i} = Adg(invg(g_sc{i})); % Adg(invg(g_oc{i}))*Adg(invg(g_po))*Adg(invg(g_sp{i}))
end
J_h = blkdiag(B_c'*Ad_inv_g_sc{1}*J_sf_s{1},B_c'*Ad_inv_g_sc{2}*J_sf_s{2});