% force-closure - check whether origin is in the convex hull of G
close all
clear

import poly.*
%% frictionless point contact (4 fingers) page 227 
% force-closure can be checked by convexity condition (props. 5.3)
g_oc1 = transl(-1,1/4,0)*trotz(-pi/2); % a=0.25, b=0.5 (rectangle: 4bx4a)
g_oc2 = transl(-1/2,1/2,0)*trotz(-pi);
g_oc3 = transl(1,-1/4,0)*trotz(pi/2);
g_oc4 = transl(1/2,-1,0);
ex = [1,0,0,0,0,0]';
ey = [0,1,0,0,0,0]';
G = [Adg(invg(g_oc1))'*ey, Adg(invg(g_oc2))'*ey, Adg(invg(g_oc3))'*ey, Adg(invg(g_oc4))'*ey];
%% point contact with friction (2 fingers) page 243/249
g_oc1 = transl(-1,0,0)*trotz(-pi/2);
g_oc2 = transl(1,0,0)*trotz(pi/2);
B_c1 = [ex,ey]; B_c2 = B_c1;
G = [Adg(invg(g_oc1))'*B_c1, Adg(invg(g_oc2))'*B_c2]; % it cannot be checked directly by props. 5.3
% note that all contact forces which lie in the friction cone can be written as 
% positive linear combinations of the forces which describe the edges of the cone.
epos = [1/2,1,0,0,0,1/2]'; % r=1, mu=0.5 (rectangle: 2rxr)
eneg = [-1/2,1,0,0,0,-1/2]';
B_c1 = [epos,eneg]; B_c2 = B_c1;
G = [Adg(invg(g_oc1))'*B_c1, Adg(invg(g_oc2))'*B_c2];
%% planar antipodal grasp page 253
% point contact with friction (2 fingers) (rectangle page 243)
syms y1 y2 real;
p1 = [-1;y1];
p2 = [1;y2];
n1 = [1,0]';
n1_perp = [0,1]';
n2 = [-1,0]';
n2_perp = [0,-1]';
mu = 0.5;
fa = -det([n1-mu*n1_perp,p2-p1]);
fb = det([n1+mu*n1_perp,p2-p1]);
fc = -det([n2-mu*n2_perp,p1-p2]);
fd = det([n2+mu*n2_perp,p1-p2]);
[y1, y2] = meshgrid(-0.5:0.01:0.5, -0.5:0.01:0.5); 
f1 = y1 - y2 - 1;
f2 = y2 - y1 - 1;
f = max(f1,f2);
y1(f>0)=nan;
y2(f>0)=nan;
figure
plot(y1(:),y2(:));grid;hold on
fimplicit(@(y1,y2) y1 - y2 - 1,[-2,2]);
fimplicit(@(y1,y2) y2 - y1 - 1,[-2,2]);
%% visualization
xyt = G([1,2,6],:)';
in = inhull([0,0,0],xyt);
face = [1,2,4;
    1,2,3;
    2,3,4;
    1,3,4];
figure
hold on
view(-38,28); hold on
patch('Vertices',xyt,'Faces',face,'FaceColor','y','FaceAlpha',0.5);
plot3(xyt(:,1),xyt(:,2),xyt(:,3),'bo');
plot3(0,0,0,'ro');
hold off