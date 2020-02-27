close all
clear

x0 = 1; y0 = 2;
[x,y] = meshgrid((-2:.1:2)+x0,(-2:.1:2)+y0);
z = (x.^2-x0^2*ones(size(x))).^2+(y.^2-y0^2*ones(size(y))).^2;
surf(x,y,z);