close all
clear


import SE3.*
% D-H params (page 114, toolbox page 216)
% if you use D-H params, the i-th frame should be in the end of the i-th link
% g = SE3(transl([0,0,d])*trotz(th)*transl([a,0,0])*trotx(alpha))
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
% DH to PoE (R. Murray 1994)
xi_R = [0,0,0,0,0,1]';
xi_P = [0,0,1,0,0,0]';
d = 0.4; th = 0; a = 0.25; alpha = 0;
gl_01 = transl([0,0,d])*trotz(th)*transl([a,0,0])*trotx(alpha);
d = 0; th = 0; a = 0.15; alpha = pi;
gl_12 = transl([0,0,d])*trotz(th)*transl([a,0,0])*trotx(alpha);
d = 0; th = 0; a = 0; alpha = 0;
gl_23 = transl([0,0,d])*trotz(th)*transl([a,0,0])*trotx(alpha);
d = 0.075; th = 0; a = 0; alpha = 0;
gl_34 = transl([0,0,d])*trotz(th)*transl([a,0,0])*trotx(alpha);
xi(:,1) = xi_R;
xi(:,2) = Adg(gl_01)*xi_R;
xi(:,3) = Adg(gl_01*gl_12)*xi_P;
xi(:,4) = Adg(gl_01*gl_12*gl_23)*xi_R;
g_st = expm(hatwedge(xi(:,1))*rth(1))*expm(hatwedge(xi(:,2))*rth(2))*expm(hatwedge(xi(:,3))*rth(3))*expm(hatwedge(xi(:,4))*rth(4))*gl_01*gl_12*gl_23*gl_34;
% PoE to DH (L. Wu 2017)

