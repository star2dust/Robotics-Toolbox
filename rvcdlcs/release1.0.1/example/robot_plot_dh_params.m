% robot plot by D-H parameters
% Reference book: Richard M. Murray-A Mathematical Introduction to Robotic Manipulation-CRC Press (1994).
% Reference paper: Hernández-Barragán, J., López-Franco, C., Alanis, A. Y., Arana-Daniel, N., & López-Franco, M. (2019). Dual-arm cooperative manipulation based on differential evolution. International Journal of Advanced Robotic Systems. https://doi.org/10.1177/1729881418825188

close all
clear
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
%% KUKA youbot
link = [ Revolute('d',0.147,'a',0.033,'alpha',pi/2), Revolute('a', 0.155), Revolute('a', 0.135), Revolute('alpha', pi/2), Revolute('d', 0.2175)];
figure
youbot = SerialLink(link, 'name', 'youbot');
rth = [0.5,0.5,0.5,0.5,0.5];
youbot.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);
%% Puma 560
link = [ Revolute('alpha',pi/2), Revolute('a', 0.4318), Revolute('d', 0.15, 'a', 0.0203, 'alpha', -pi/2), Revolute('d', 0.4318, 'alpha', pi/2), Revolute('alpha', -pi/2), Revolute()];
figure
puma = SerialLink(link, 'name', 'puma');
rth = [0.5,0.5,0.5,0.5,0.5,0.5];
puma.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);
%% Stanford
link = [ Revolute('d', 0.412, 'alpha', -pi/2), Revolute('d', 0.154, 'alpha', pi/2), Prismatic('qlim',[0 0.5]), Revolute('alpha', -pi/2), Revolute('alpha', pi/2), Revolute('d', 0.263)];
figure
stanford = SerialLink(link, 'name', 'stanford');
rth = [0.5,0.5,0.1,0.5,0.5,0.5];
stanford.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);
%% Baxter 
link = [ Revolute('d', 0.27035, 'a', 0.069, 'alpha', -pi/2), Revolute('alpha', pi/2), Revolute('d', 0.36435, 'a', 0.069, 'alpha', -pi/2), Revolute('alpha', pi/2), Revolute('d', 0.37429, 'a', 0.010, 'alpha', -pi/2), Revolute('a', 0.010, 'alpha', pi/2), Revolute('d', 0.229525, 'a', 0.010)];
figure
baxter = SerialLink(link, 'name', 'baxter');
rth = [0.5,0.5,0.5,0.5,0.5,0.5,0.5];
baxter.plot(rth,'workspace',[-1 1 -1 1 -1 1], 'tilesize', 0.5);