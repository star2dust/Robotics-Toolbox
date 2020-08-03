close all
clear

import PlanarRevolute.*
m = 2; l = ones(m,1)*0.8; ed = [1,.8,.5]; mh = [0.2,0.2,1];
Qlim = [zeros(m,1),ones(m,1)*pi/2];
cub_surf= [mh(1),mh(2),ed(3)/2+ed(1)/6]; hb_d = mh(3); 
% choose q according to type of joints
xi = getTwist(l,hb_d,SE3);
% poe => dh modified
[dh, Ht, sigma] = poe2dh(xi,hb_d); % qb only for theta or d
Hb = transl(cub_surf); % Hb can only be on the surface of mobile base
rob = MobileRobot(ed,xi,Hb,'name','rob1','qlim', Qlim);
rob.plot([0,0,0,0,0],'frame','workspace',[-1,3,-1,3,0,4]);
q0 = [0,0,0,0,0];
q1 = [1,0,-1,1,2];
q2 = [1,1,1,2,1];
[qq,~,~,tq] = mstraj_([q0;q1;q2],0.1*ones(size(q0)),0.1,1);
tic;
playspeed = 2;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    rob.animate(q);
    drawnow
end
toc