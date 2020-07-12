close all
clear

% D-H params (page 114, toolbox page 216)
% if you use D-H params, the i-th frame should be in the end of the i-th link
% g = SE3(trotz(th)*transl([0,0,d])*trotx(alpha)*transl([a,0,0]))
% see also https://robotics.stackexchange.com/questions/4364/denavit-hartenberg-parameters-for-scara-manipulator
%% SCARA robot
link = [ Revolute('d',0.4,'a',0.25), Revolute('a', 0.15, 'alpha', pi), Prismatic('qlim', [0 0.325]), Revolute('d', 0.075)];
% SCARA construction and forward kinematics (toolbox page 219)
% SerialLink object cannot use list
figure
scara = SerialLink(link, 'name', 'scara');
rth = [0.5,0.5,0.2,0.5];
scara.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);
scara.fkine(rth)
%% DH to PoE (R. Murray 1994)
dh = [0,0.4,0.25,0;
    0,0,0.15,pi;
    0,0,0,0;
    0,0.075,0,0];
xi = dh2poe(dh,'RRPR');
g_st = expm(wedge(xi(:,1))*rth(1))*expm(wedge(xi(:,2))*rth(2))*expm(wedge(xi(:,3))*rth(3))*expm(wedge(xi(:,4))*rth(4))*expm(wedge(xi(:,5)));
%% DH to PoE (L. Wu 2017)
% xi2 = DH2POE( dh, eye(4), eye(4), 'RRPR', 'std');
%% PoE to DH (L. Wu 2017)
qb = 0.4; % stdDH([0,0.4,0,0]); % only theta (P) and d (R)
[dh3,Ht,sigma] = poe2dh(xi,qb);
qlim = [0,pi/2;0,pi/2;0,0.5;0,pi/2];
% dh3 = [  0         0    0.2500         0
%          0         0         0    3.1416
%     0.0000         0    0.1500         0
%          0   -0.3250    0.0000    0.0000];
figure
scara2 = SerialLink([dh3,sigma], 'name', 'scara2', 'tool', Ht, 'qlim', qlim);
rth = [0.5,0.5,0.2,0.5];
scara2.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);
scara2.fkine(rth)
