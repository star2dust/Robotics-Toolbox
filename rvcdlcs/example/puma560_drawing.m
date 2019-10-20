% PUMA560 robot draws a letter on a surface
% Reference book: Peter Corke - Robotics, Vision and Control (2017) p239
close all
clear

%% generate path of character 'B'
load hershey;
B_word = hershey{'B'};
% to scale the path by 0.25 so that the character is around 20 cm tall
path = [ 0.5*B_word.stroke; zeros(1,numcols(B_word.stroke))];
% B_word.stroke: the rows are the x- and y-coordinates respectively, and a column of NaNs indicates the end of a segment
% find the columns that contain NaNs and replace them with the preceding column 
k = find(isnan(path(1,:)));
% the z-coordinate is set to 0.2 in order to lift the pen off the surface
path(:,k) = path(:,k-1); path(3,k) = 0.2;
%% generate trajectory
% the second argument is the maximum speed in the x-, y- and z-directions, the fourth argument is the initial coordinate followed by the sample interval and the acceleration time. 
traj = mstraj(path(:,2:end)', [0.5 0.5 0.5], [], path(:,1)',0.02, 0.2);
% test with p560 (az = -38, el = 28)
mdl_puma560;
Tp = SE3(0.4, 0, 0) * SE3(traj) * SE3.oa( [0 1 0], [0 0 -1]);
q = p560.ikine6s(Tp);
p = transl(p560.fkine(q));
p560.plot(q(1,:));
plot3(p(:,1),p(:,2),p(:,3),'r'); 
p560.plot(q);