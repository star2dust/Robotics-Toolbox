close all
clear

[x,y] = meshgrid(-1:.1:1,-2:.1:2);
z1 = x.*(cos(y)-sin(y));
z2 = x.*(sin(y)+cos(y));
surf(x,y,z1);
figure
surf(x,y,z2);