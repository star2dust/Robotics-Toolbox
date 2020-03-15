close all
clear


l = 1; 
f = @(q) (l*cos(q(1))-l/(tan(q(2))+1))^2/2+(l*sin(q(1))-l/(tan(q(2))+1)*tan(q(2)))^2/2;
[q_min, f_min] = fmincon(f,[0;0],[],[],[],[],[0;0],[pi/2;pi/2]-.01);

[th,alpha] = meshgrid(0:.1:pi/2,0:.1:pi/2);
x = l./(tan(alpha)+1);
z = (l*cos(th)-x).^2/2+(l*sin(th)-x*tan(alpha)).^2/2;
surf(th,alpha,z);
ylabel('alpha');xlabel('th');zlabel('z');


[th,x] = meshgrid(0:.1:pi/2,0:.1:1);
z = (l*cos(th)-x).^2/2+(l*sin(th)-(l-x)).^2/2;
surf(th,x,z);
ylabel('x');xlabel('th');zlabel('z');
% x = l/(tan(alpha)+1);
% f = @(th) (l*cos(th)-x)^2/2+(l*sin(th)-x*tan(alpha))^2/2;
% [th_min, f_min] = fmincon(f,0,[],[],[],[],0,pi/2);