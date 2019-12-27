close all
clear


import SE3.*
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
scara.fkine(rth);
%% DH to PoE (R. Murray 1994)
dh = [0,0.4,0,0.25;
    0,0,pi,0.15;
    0,0,0,0;
    0,0.075,0,0];
xi = dh2poe(dh,'RRPR');
g_st = expm(hatwedge(xi(:,1))*rth(1))*expm(hatwedge(xi(:,2))*rth(2))*expm(hatwedge(xi(:,3))*rth(3))*expm(hatwedge(xi(:,4))*rth(4))*expm(hatwedge(xi(:,5)));
%% DH yp PoE (L. Wu 2017)
% xi2 = DH2POE( dh, eye(4), eye(4), 'RRPR', 'std');
%% PoE to DH (L. Wu 2017)
dh3 = poe2dh(xi);

link = [ Revolute('a', 0.25), Revolute('alpha',pi), Prismatic('qlim', [0 0.325], 'a', 0.15), Revolute('d',-0.325)];
figure
scara2 = SerialLink(link, 'name', 'scara2');
rth = [0.5,0.5,0.2,0.5];
scara2.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);
scara2.fkine(rth)