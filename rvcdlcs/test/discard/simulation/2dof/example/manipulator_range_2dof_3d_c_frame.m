close all
clear
 
% % define mR manipulator and calculate manipulability
% m = 2; % mR manipulator (m dof)
% th = sym('th',[m,1],'real');
% l = sym('l',[m,1],'real');
% [mpe,J] = mRfkine(th,l);
% mu = simplify(sqrt(det(J*J')));
% display(mu);
% 
% % get q_intsec with best manipulability
% link = ones(m,1)*0.5; % each link length = 0.5
% epmcbar = [0;0]+[1;0]*link(2)+[cos(pi/3);sin(pi/3)]*link(1);
% [epcx,epcy,th2] = meshgrid(-.6:.01:.6,-.6:.01:.6,-1.6:.01:0);
% fx = (epcx+cos(-th2)*link(1)+link(2))-epmcbar(1);
% fy = (epcy+sin(-th2)*link(1))-epmcbar(2);
% negmupower2 = @(q) -(link(1)*link(2))^2*sin(q(3))^2;
% q_intsec = fmincon(negmupower2,[0;0;-pi/2],[],[],[],[],[-.5;-.5;-pi/2],[.5;.5;0],@(q) intsectcons([q;-epmcbar],link));

% plot original joint space
figure
[px,py,th2] = meshgrid(-0.1:.02:2.1,-0.1:.02:2.1,-1.6:.02:1.6);
cxl = px-2;
cxu = -px;
cx = max(cxl,cxu);
cyl = py-2;
cyu = -py-2;
cy = max(cyl,cyu);
% theta constraints
phi = 0;
cthl = phi-th2-pi/2;
cthu = -phi+th2;
cth = max(cthl,cthu);
cons = max(cx,max(cy,cth));
% error constraints
m = 2; link = ones(m,1)*1; % each link length = 0.5
phil = 0;
Rl = rot2(phil)';
cerrx = -(Rl(1,1)*px+Rl(1,2)*py-(link(1)*cos(phi-th2)+link(2)*cos(phi)));
cerry = -(Rl(2,1)*px+Rl(2,2)*py-(link(1)*sin(phi-th2)+link(2)*sin(phi)));
cons = max(cerrx,max(cerry,cons));
facevert = isosurface(px,py,th2,cons,0);
pat = patch(facevert);
isonormals(px,py,th2,cons,pat)
set(pat,'facecolor',[0 .5 1],'edgecolor','none');hold on
view(3),axis([0 2 -2 2 -3.2 3.2]),grid on
ylabel('Y');xlabel('X');zlabel('TH');
camlight
lighting gouraud
hold off
for phi=0:0.1:pi/2
    % theta constraints
    cthl = phi-th2-pi/2;
    cthu = -phi+th2;
    cth = max(cthl,cthu);
    cons = max(cx,max(cy,cth));
    % error constraints
    cerrx = -(Rl(1,1)*px+Rl(1,2)*py-(link(1)*cos(phi-th2)+link(2)*cos(phi)));
    cerry = -(Rl(2,1)*px+Rl(2,2)*py-(link(1)*sin(phi-th2)+link(2)*sin(phi)));
    cons = max(cerrx,max(cerry,cons));
    facevert = isosurface(px,py,th2,cons,0);
    set(pat,'faces',facevert.faces,'vertices',facevert.vertices);
    isonormals(px,py,th2,cons,pat)
    pause(0.01)
end

% % modify joint space to convex space
% dfx1 = Rl(1,1); dfx2 = Rl(1,2); dfx3 = sin(-q_intsec(3))*link(1);
% dfy1 = Rl(2,1); dfy2 = Rl(2,2); dfy3 = -cos(-q_intsec(3))*link(1);
% % normal vector
% nx = [dfx1;dfx2;dfx3];
% ny = [dfy1;dfy2;dfy3];
% % tangent plane
% ftx = nx(1)*(px-q_intsec(1))+nx(2)*(py-q_intsec(2))+nx(3)*(th2-q_intsec(3));
% fty = ny(1)*(px-q_intsec(1))+ny(2)*(py-q_intsec(2))+ny(3)*(th2-q_intsec(3));
% % add other constraints
% figure
% cons = max(cx,max(cy,cth));
% % cons = max(cx,max(cy,max(cth,max(ftx,fty))));
% facevert = isosurface(px,py,th2,cons,0);
% pat = patch(facevert);
% isonormals(px,py,th2,cons,pat)
% set(pat,'facecolor',[0 .5 1],'edgecolor','none');hold on
% view(150,30),axis image,grid on
% ylabel('Y');xlabel('X');zlabel('Z');
% camlight
% lighting gouraud
% hold on
% plot3(q_intsec(1),q_intsec(2),q_intsec(3),'ro');
% 
% % A and b
% lb = [-.5;-.5;-pi/2];
% ub = [.5;.5;-asin(0.2)];
% A1 = nx'; b1 = -nx'*q_intsec;
% A2 = ny'; b2 = -ny'*q_intsec;
% A = [A1;A2];
% b = [b1;b2];
% % proj
% px = [-0.2;0.4;-0.8];
% pj = lsqlin(eye(length(px)),px,A,-b,[],[],lb,ub);
% plot3([px(1) pj(1)],[px(2) pj(2)],[px(3) pj(3)],'ro-');

