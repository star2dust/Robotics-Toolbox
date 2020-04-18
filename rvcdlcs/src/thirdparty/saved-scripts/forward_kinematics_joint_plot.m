close all
clear
clc

% Plot the joints when forward kinematics f<=[2;2]
% f1 = @(x,y,z) 2-(cos(x + y + z) + cos(x + y) + cos(x) + 1/2);
% f2 = @(x,y,z) 2.5-(sin(x + y + z) + sin(x + y) + sin(x));
% figure
% ezimplot3(f1,[-pi pi]);
% figure
% ezimplot3(f2,[-pi pi]);

figure
[x,y,z] = meshgrid(0:.01:2,-1.5:.01:1.5,-1.5:.01:1.5);
f1 = 2-(cos(x + y + z) + cos(x + y) + cos(x) + 1/2);
f2 = 2-(sin(x + y + z) + sin(x + y) + sin(x));
c1 = x+y+z-pi/2;
c2 = x+y-pi/2;
c3 = -x;
c4 = 30-(cos(x - z)/2 - cos(2*x + y)/4 - cos(x + 2*y)/2 + 2*cos(2*x + z) - 2*cos(2*y + z) + (7*cos(2*x))/4 - cos(2*y) - 9*cos(2*x + 2*y + 2*z) - cos(3*x + 2*y + 2*z) - cos(4*x + 2*y + 2*z) - cos(2*x + 2*y)/4 - cos(2*y + 2*z) - (3*cos(x + 2*y + z))/2 - (9*cos(2*x + y + z))/2 + cos(x + y) + (3*cos(x + z))/2 + (9*cos(y + z))/2 + (9*cos(x))/2 + (17*cos(y))/4 + 9*cos(z) - cos(x + 2*y + 2*z) - 9*cos(2*x + 2*y + z) - cos(3*x + 2*y + z)/2 + 89/4);
f = max(f1,f2);% c11 = c1<=0 & c2<=0 & c3<=0;
c = max(max(c1,c2),c3);
fv1 = isosurface(x,y,z,f,0);
fv2 = isosurface(x,y,z,c,0);
fv3 = isosurface(x,y,z,c4,0);
p = patch(fv1);
isonormals(x,y,z,c,p)
set(p,'facecolor',[0 .5 1],'edgecolor','none');hold on
view(150,30),axis image,grid on
ylabel('Y');xlabel('X');zlabel('Z');
camlight
lighting gouraud
patch(fv2,'edgecolor','none','facecolor','r','facealpha',0.3);
patch(fv3,'edgecolor','none','facecolor','y','facealpha',0.3);