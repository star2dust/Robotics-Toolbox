close all
clear

% get q_intsec with best manipulability
m = 2; % mR manipulator (m dof)
link = ones(m,1)*0.5; % each link length = 0.5
negmupower2 = @(q) -(link(1)*link(2))^2*sin(q(3))^2;
lb = [-.5;-.5;-pi/2;-1.25;-.9];
ub = [.5;.5;-asin(0.6);-.2;0];
% initial states q = [epcx,epcy,th2,epmcx,epmcy]
th20 = -pi/4;
q0 = [0;0;th20;-[link(2)+link(1)*cos(-th20);link(1)*sin(-th20)]];
q_intsec = fmincon(negmupower2,q0,[],[],[],[],lb,ub,@(q) intsectcons(q,link));

% modify joint space to convex space
dfx1 = 1; dfx2 = 0; dfx3 = sin(-q_intsec(3))*link(1); dfx4 = 1; dfx5 = 0;
dfy1 = 0; dfy2 = 1; dfy3 = -cos(-q_intsec(3))*link(1); dfy4 = 0; dfy5 = 1;
% normal vector
nx = [dfx1;dfx2;dfx3;dfx4;dfx5];
ny = [dfy1;dfy2;dfy3;dfy4;dfy5];
% tangent plane
A = [nx,ny]'; b = A*q_intsec;

% projection => Solve constrained linear least-squares problems
px = [0.2;-0.4;-0.8;1;1];
pj = lsqlin(eye(length(px)),px,A,b,[],[],lb,ub);


% plot
epmcbar = pj(4:5);
[epcx,epcy,th2] = meshgrid(-.6:.01:.6,-.6:.01:.6,-1.6:.01:0);
[epmcx,epmcy] = meshgrid(.2:.01:1.25,0:.01:.9);
fx = (epcx+cos(-th2)*link(1)+link(2))-epmcbar(1);
fy = (epcy+sin(-th2)*link(1))-epmcbar(2);

% plot joint space
figure
cxl = -epcx-.5;
cxu = epcx-.5;
cx = max(cxl,cxu);
cyl = -epcy-.5;
cyu = epcy-.5;
cy = max(cyl,cyu);
cthl = -th2-pi/2;
cthu = th2+asin(0.6);
cth = max(cthl,cthu);
% modify joint space to convex space
dfx1 = 1; dfx2 = 0; dfx3 = sin(-q_intsec(3))*link(1);
dfy1 = 0; dfy2 = 1; dfy3 = -cos(-q_intsec(3))*link(1);
% normal vector
nx = [dfx1;dfx2;dfx3];
ny = [dfy1;dfy2;dfy3];
% tangent plane
ftx = nx(1)*(epcx-q_intsec(1))+nx(2)*(epcy-q_intsec(2))+nx(3)*(th2-q_intsec(3));
fty = ny(1)*(epcx-q_intsec(1))+ny(2)*(epcy-q_intsec(2))+ny(3)*(th2-q_intsec(3));
cons = max(cx,max(cy,max(cth,max(ftx,fty))));
facevert = isosurface(epcx,epcy,th2,cons,0);
pat = patch(facevert);
isonormals(epcx,epcy,th2,cons,pat)
set(pat,'facecolor',[0 .5 1],'edgecolor','none','facealpha',0.5);hold on
view(150,30),axis image,grid on
ylabel('Y');xlabel('X');zlabel('TH');
camlight
lighting gouraud
hold on
q_line = [px(1:3),pj(1:3)];
plot3(q_line(1,:),q_line(2,:),q_line(3,:),'ro');

% plot work space
figure
m_cons = [lb(4),lb(4),ub(4),ub(4),lb(4);
    lb(5),ub(5),ub(5),lb(5),lb(5)];
patch('XData', m_cons(1,:),'YData', m_cons(2,:),'edgecolor','none','facecolor',[0 .5 1],'facealpha',0.5);
hold on
m_line = [px(4:5),pj(4:5)];
plot(m_line(1,:),m_line(2,:),'ro');
ylabel('Y');xlabel('X');
