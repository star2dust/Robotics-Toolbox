close all
clear

[x,y] = meshgrid(0:pi/50:pi/2,0:pi/50:pi/2);
z1 = cos(x)+cos(x+y); % cos(x)+cos(x+y)-z<0
z1(x+y>pi/2)=nan;
x0 = 0.5; y0 = 0.5; z0 = cos(x0)+cos(x0+y0);
z2 = (-sin(x0)-sin(x0+y0))*(x-x0)-sin(x0+y0)*(y-y0)+z0; %(-sin(x0)-sin(x0+y0))*(x-x0)-sin(x0+y0)*(y-y0)-(z-z0)<0
surf(x,y,z1); hold on
surf(x,y,z2)
z1 = sin(x)+sin(x+y);
z1(x+y>pi/2)=nan;
surf(x,y,z1);