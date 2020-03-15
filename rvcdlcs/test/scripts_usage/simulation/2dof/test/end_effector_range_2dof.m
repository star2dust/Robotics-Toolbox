clear
close all

m = 2;
[epcx,epcy,th2] = meshgrid(-.5:.01:.5,-.5:.01:.5,-pi/2:.01:asin(0.6));
q_intsec = [0.2492;-0.0670;-1.5691];
link = ones(m,1)*0.5;

% bound constraints
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
% add other constraints
% cons = max(cx,max(cy,max(cth,max(ftx,fty))));
cons = max(cx,max(cy,cth));

% range of mpc
figure
px = epcx+cos(-th2)*link(1)+link(2);
py = epcy+sin(-th2)*link(1);
px(cons>0)=nan;
py(cons>0)=nan;
plot(px(:),py(:),'y.')
