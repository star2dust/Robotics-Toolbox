close all
clear

% generate meshgrid
[x,y] = meshgrid(-2:.1:2,-2:.1:2);

%% projection to a line 
f_line = zeros(size(x));
a = [0.6;0.8];
P_a = a*a';
for i=1:length(x(:))
    q = [x(i);y(i)]; 
    f_line(i) = (q-P_a*q)'*P_a*q;
end
figure
surf(x,y,f_line);
axis([-2 2 -2 2 -2 2])

%% projection to a circle
f_circ = zeros(size(x));
u = rand(2,1)*10-5;
p0 = [0;0]; r = 1;
for i=1:length(x(:))
    p = [x(i);y(i)];
    pj = proj_circle(p,p0,r);
    f_circ(i) = (p-pj)'*pj;
%     f_circ(i) = (p-pj)'*(proj_circle(p-u,p0,r)-pj);
end
figure
surf(x,y,f_circ);
xlabel('x');ylabel('y');zlabel('z');

% calculate partial derivative
syms X Y real
% u = [1;1];
p = [X;Y]; 
pj = r*(p-p0)/norm(p-p0);
% puj = r*(p-u-p0)/norm(p-u-p0);
% F = (p-pj)'*(puj-pj);
F = (p-pj)'*pj;
dfX = diff(F,X);
dfY = diff(F,Y);