close all
clear

% x = 0:0.01:1;
% th = atan(x.*sin(pi/3)./(1-x.*cos(pi/3)));
% y = 1-cos(th)-x;
% plot(x,y)

th = pi/6:0.001:pi/3;
y2 = 6*(tan(th-pi/6)*cos(pi/6)+sin(pi/6));
z2 = 4*cos(th)+2;
y = 5*(tan(th-pi/6)*cos(pi/6)+sin(pi/6));
z = (cos(th)+sqrt(4-sin(th).^2))+2;
plot(th,y); hold on
plot(th,z);

syms th
y = 6*(tan(th-pi/6)*cos(pi/6)+sin(pi/6));
z2 = (4-0.2)*cos(th)+2+0.2;
x0 = solve(y-z2, 'Real', true);
eval(x0)
