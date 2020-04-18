close all
clear
 
import mpr.*
% define mR manipulator and calculate manipulability
m = 2; % mR manipulator (m dof)
th = sym('th',[m,1],'real');
l = sym('l',[m,1],'real');
[mpe,J] = fkine(th,l);
mu = simplify(sqrt(det(J*J')));
display(mu);

% get q_intsec with best manipulability
link = ones(m,1)*0.5; % each link length = 0.5
epmcbar = [0;0]+[1;0]*link(2)+[cos(pi/3);sin(pi/3)]*link(1);
[epcx,epcy,th2] = meshgrid(-.6:.01:.6,-.6:.01:.6,-1.6:.01:0);
fx = (epcx+cos(-th2)*link(1)+link(2))-epmcbar(1);
fy = (epcy+sin(-th2)*link(1))-epmcbar(2);
negmupower2 = @(q) -(link(1)*link(2))^2*sin(q(3))^2;
q_intsec = fmincon(negmupower2,[0;0;-pi/2],[],[],[],[],[-.5;-.5;-pi/2],[.5;.5;0],@(q) intsectcons([q;-epmcbar],link));

% plot original joint space
figure
cxl = -epcx-.5;
cxu = epcx-.5;
cx = max(cxl,cxu);
cyl = -epcy-.5;
cyu = epcy-.5;
cy = max(cyl,cyu);
cthl = -th2-pi/2;
cthu = th2+asin(0.2);
cth = max(cthl,cthu);
cons = max(cx,max(cy,max(cth,max(fx,fy))));
facevert = isosurface(epcx,epcy,th2,cons,0);
pat = patch(facevert);
isonormals(epcx,epcy,th2,cons,pat)
set(pat,'facecolor',[0 .5 1],'edgecolor','none');hold on
view(150,30),axis image,grid on
ylabel('Y');xlabel('X');zlabel('Z');
camlight
lighting gouraud
hold on
plot3(q_intsec(1),q_intsec(2),q_intsec(3),'ro');

% modify joint space to convex space
dfx1 = 1; dfx2 = 0; dfx3 = sin(-q_intsec(3))*link(1);
dfy1 = 0; dfy2 = 1; dfy3 = -cos(-q_intsec(3))*link(1);
% normal vector
nx = [dfx1;dfx2;dfx3];
ny = [dfy1;dfy2;dfy3];
% tangent plane
ftx = nx(1)*(epcx-q_intsec(1))+nx(2)*(epcy-q_intsec(2))+nx(3)*(th2-q_intsec(3));
fty = ny(1)*(epcx-q_intsec(1))+ny(2)*(epcy-q_intsec(2))+ny(3)*(th2-q_intsec(3));
% add other constraints
figure
cons = max(cx,max(cy,max(cth,max(ftx,fty))));
facevert = isosurface(epcx,epcy,th2,cons,0);
pat = patch(facevert);
isonormals(epcx,epcy,th2,cons,pat)
set(pat,'facecolor',[0 .5 1],'edgecolor','none');hold on
view(150,30),axis image,grid on
ylabel('Y');xlabel('X');zlabel('Z');
camlight
lighting gouraud
hold on
plot3(q_intsec(1),q_intsec(2),q_intsec(3),'ro');

% A and b
lb = [-.5;-.5;-pi/2];
ub = [.5;.5;-asin(0.2)];
A1 = nx'; b1 = -nx'*q_intsec;
A2 = ny'; b2 = -ny'*q_intsec;
A = [A1;A2];
b = [b1;b2];
% proj
px = [-0.2;0.4;-0.8];
pj = lsqlin(eye(length(px)),px,A,-b,[],[],lb,ub);
plot3([px(1) pj(1)],[px(2) pj(2)],[px(3) pj(3)],'ro-');

