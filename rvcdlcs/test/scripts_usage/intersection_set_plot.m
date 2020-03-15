close all
clear
clc

% Plot the intersection of constraint sets
% mu = cos(x + y) - cos(2*x + y)/4 - cos(2*x)/4 - cos(2*y) - cos(2*x + 2*y)/4 - cos(x + 2*y)/2 + (5*cos(x))/2 + (17*cos(y))/4 + 53/4;

% [x,y,z] = meshgrid(-0.5:.01:2,-2:.01:2,-2:.01:2);
[x,y,z] = meshgrid(0:.01:0.8,0.3:.01:1,-0.5:.01:1);
c1 = x+y+z-pi/2;
c10 = -x-y-z;
c2 = x+y-pi/2;
c20 = -x-y;
c3 = -x;
c30 = x-pi/2;
c4 = 19-(cos(x + y) - cos(2*x + y)/4 - cos(2*x)/4 - cos(2*y) - cos(2*x + 2*y)/4 - cos(x + 2*y)/2 + (5*cos(x))/2 + (17*cos(y))/4 + 53/4);
f1 = -2+(cos(x + y + z) + cos(x + y) + cos(x) + 1/2);
f2 = -2.5+(sin(x + y + z) + sin(x + y) + sin(x));
c = max(max(max(max(max(c1,c2),c3),c4),f1),f2);% c11 = c1<=0 & c2<=0 & c3<=0;
% c = max(max(max(c1,c2),c3),c4);% c11 = c1<=0 & c2<=0 & c3<=0;
fv1 = isosurface(x,y,z,c,0);
p = patch(fv1);
isonormals(x,y,z,c,p)
set(p,'facecolor',[0 .5 1],'edgecolor','none');hold on;
view(150,30),axis image,grid on
ylabel('Y');xlabel('X');zlabel('Z');
camlight
lighting gouraud
c40 = 19-(cos(x + y) - cos(2*x + y)/4 - cos(2*x)/4 - cos(2*y) - cos(2*x + 2*y)/4 - cos(x + 2*y)/2 + (5*cos(x))/2 + (17*cos(y))/4 + 53/4);
fv2 = isosurface(x,y,z,c40,0);
patch(fv2,'edgecolor','none','facecolor','r','facealpha',0.3);

f1 = 2-(cos(x + y + z) + cos(x + y) + cos(x) + 1/2);
f2 = 2.5-(sin(x + y + z) + sin(x + y) + sin(x));
f = max(f1,f2);
fv3 = isosurface(x,y,z,f,0);
patch(fv3,'edgecolor','none','facecolor','y','facealpha',0.3);

% figure;
% c0 = max(max(max(max(max(c1,c2),c3),c10),c20),c30);
% fv2 = isosurface(x,y,z,c0,0);
% p = patch(fv2);
% isonormals(x,y,z,c0,p)
% set(p,'facecolor',[0 .5 1],'edgecolor','none')
% view(150,30),axis image,grid on
% ylabel('Y');xlabel('X');zlabel('Z');
% camlight
% lighting gouraud