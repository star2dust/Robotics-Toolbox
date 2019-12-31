% SCARA robot draws a letter on a surface
% Reference book: Peter Corke - Robotics, Vision and Control (2017) p239
close all
clear

import scarafunc_1_0_5.*
%% generate path of character 'B'
load hershey;
B_word = hershey{'B'};
% to scale the path by 0.25 so that the character is around 20 cm tall
path = [ 0.4*B_word.stroke; zeros(1,numcols(B_word.stroke))];
% B_word.stroke: the rows are the x- and y-coordinates respectively, and a column of NaNs indicates the end of a segment
% find the columns that contain NaNs and replace them with the preceding column 
k = find(isnan(path(1,:)));
% the z-coordinate is set to 0.2 in order to lift the pen off the surface
path(:,k) = path(:,k-1); path(3,k) = 0.2;
%% parameters
% link length
l1 = 0.4; l2 = 0.4; l3 = 0.4; l4 = 0; d = 0.1;
% link center of mass
r1 = l1/2; r2 = l2/2; r3 = l3/2;
% joint angle
th1 = 0; th2 = 0; th3 = 0; th4 = 0;
% link mass
m1 = 1; m2 = 1; m3 = 1; m4 = m3;
% link inertia
Iz1 = m1*(3*(d/2)^2+l1^2)/12; Iy1 = Iz1; Ix1 = m1*(d/2)^2/2;
Iz2 = m2*(3*(d/2)^2+l2^2)/12; Iy2 = Iz2; Ix2 = m2*(d/2)^2/2;
Ix3 = m3*(3*(d/2)^2+l3^2)/12; Iy3 = Ix3; Iz3 = m3*(d/2)^2/2;
Ix4 = Ix3; Iy4 = Iy3; Iz4 = Iz3;
% choose q according to type of joints
q1 = [0,0,0]';
q2 = [l1,0,0]';
q3 = [l1+l2,0,0]';
w1 = [0,0,1]';
w2 = w1; w3 = w1; v4 = w1;
% joint twists
xi1 = [-skew(w1)*q1;w1];
xi2 = [-skew(w2)*q2;w2];
xi3 = [-skew(w3)*q3;w3];
xi4 = [v4;zeros(3,1)];
% reference config for each joint
g_sl0{1} = transl(r1,0,l3);
g_sl0{2} = transl(l1+r2,0,l3);
g_sl0{3} = transl(l1+l2,0,r3);
g_sl0{4} = transl(l1+l2,0,r3);
% make a matrix
l = [l1,l2,l3,l4];
xi = {xi1,xi2,xi3,xi4};
th = [th1,th2,th3,th4];
m = [m1,m2,m3,m4];
I = {[Ix1 Iy1 Iz1],[Ix2 Iy2 Iz2],[Ix3 Iy3 Iz3],[Ix4,Iy4,Iz4]};
% reference config
g_st0 = transl(l1+l2,0,0);
%% generate trajectory
% add reference config to the start of the path
path = [transl(g_st0),transl(g_st0)+[0,0,0.2]',path+[0.4,0,0]'];
% the second argument is the maximum speed in the x-, y- and z-directions, the fourth argument is the initial coordinate followed by the sample interval and the acceleration time. 
traj = mstraj(path(:,2:end)', [0.5 0.5 0.5], [], path(:,1)',0.02, 0.2);
% get Cartesian trajectory 
SE3traj = SE3(traj);
% inverse kinematics
thtraj = scaraikine(l,SE3traj);
p = transl(scarafkine(l,thtraj(:,:,1)));
%% robot figure
figure
% plot trajectory
plot3(p(:,1),p(:,2),p(:,3),'r'); 
% test plot function
scaraplot(l,xi,g_sl0,thtraj); % the following is the implementation of this function
% % plot scara robot
% view(-38,28); hold on
% axis([0 1 0 1 0 1]);
% % links
% rod_x = [[0,0,0]',[1,0,0]']-[0.5,0,0]';
% rod_z = roty(pi/2)*rod_x;
% rod_x1 = rod_x*l1; rod_x2 = rod_x*l2; rod_z3 = rod_z*l3; rod_z0 = (rod_z+[0,0,0.5]')*l3;
% % set position
% ldata1 = g_sl0{1}*e2h(rod_x1);
% ldata2 = g_sl0{2}*e2h(rod_x2);
% ldata3 = g_sl0{4}*e2h(rod_z3);
% % plot
% link0 = plot3(rod_z0(1,:),rod_z0(2,:),rod_z0(3,:),'k');
% link1 = plot3(ldata1(1,:),ldata1(2,:),ldata1(3,:),'r');
% link2 = plot3(ldata2(1,:),ldata2(2,:),ldata2(3,:),'m');
% link3 = plot3(ldata3(1,:),ldata3(2,:),ldata3(3,:),'b');
% hold off
% %% robot animate
% for i=1:size(thtraj,1)
%     for j=1:size(thtraj,2)
%         g_sl{j} = g_sl0{j};
%         for k=j:-1:1
%             g_sl{j} =  expm(hatwedge(xi{k}).*thtraj(i,k,1))*g_sl{j};
%         end
%     end
%     % update position
%     ldata1 = g_sl{1}*e2h(rod_x1);
%     ldata2 = g_sl{2}*e2h(rod_x2);
%     ldata3 = g_sl{4}*e2h(rod_z3);
%     set(link1,'XData',ldata1(1,:),'YData',ldata1(2,:),'ZData',ldata1(3,:));
%     set(link2,'XData',ldata2(1,:),'YData',ldata2(2,:),'ZData',ldata2(3,:));
%     set(link3,'XData',ldata3(1,:),'YData',ldata3(2,:),'ZData',ldata3(3,:));
%     drawnow
% end
