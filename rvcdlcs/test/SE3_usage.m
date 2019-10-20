% SE3 class usage example
% reference book: Peter Corke - Robotics, Vision and Control (2017).
close all
clear

%% parameters
% link length
syms l1 l2 l3 real
% link center of mass
syms r1 r2 r3 real
% joint angle
syms th1 th2 th3 th4 real
%% reference config
g_st0 = transl(0,l1+l2,l0);
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
% make a matrix
xi = {xi1,xi2,xi3,xi4};
th = [th1,th2,th3,th4];
n = length(th);
%% SE3 usage
for i=1:n
    T{i} = expm(hatwedge(xi{i}).*th(i));
    Ts(i) = SE3(T{i});
end
about Ts % Ts [SE3] : 1x4 (32 bytes)

% Information and test methods
dim(Ts) % always 4 no matter how long the SE3 vector is
isSE(Ts) % returns true
issym(Ts) % true if rotation matrix has symbolic elements
isidentity(Ts(1)) % true for null motion
SE3.isa(Ts) % check if matrix is SO2

% Conversion methods
SE3.check(T{1}) % convert object or matrix to SE3 object