% one-link robot kinetic energy
close all
clear


% length and mass
% l = 1; m = 1; th = 0.5; dth = 0.2; r = l/2;
syms l m th dth r real
% choose q and w according to type of joints (in spacial frame see Fig. 3.3)
q = [0,0,0]';
w = [0,0,1]';

% reference config for each joint (at center)
g_sr0 = SE3(r,0,0);
% make link
link = linkstruct(m, l, 'R', q, w, 'O', g_sr0);
% g_sr
g_sr = SE3(expm(hatwedge(link.twist)*th))*g_sr0;
% body Jacobian
J_sr_b = Ad(inv(g_sr))*link.twist;
% body velocity
V_sr_b = J_sr_b*dth;
% kinetic energy
Tr = dth'*J_sr_b'*link.inertia*J_sr_b*dth/2;

% reference config for each joint (at end)
g_sl0 = SE3(l,0,0);
% make link
link = linkstruct(m, l, 'R', q, w, 'O', g_sl0);
% g_sl
g_sl = SE3(expm(hatwedge(link.twist)*th))*g_sl0;
% body Jacobian
J_sl_b = Ad(inv(g_sl))*link.twist;
% body velocity
V_sl_b = J_sl_b*dth;
% body velocity (alternative)
V_lr_b = zeros(6,1); g_lr = SE3(transl([r-l,0,0]));
V_sr_b = Ad(inv(g_lr))*V_sl_b+V_lr_b;
% kinetic energy
Tl = dth'*J_sl_b'*Ad(inv(g_lr))'*link.inertia*Ad(inv(g_lr))*J_sl_b*dth/2;


