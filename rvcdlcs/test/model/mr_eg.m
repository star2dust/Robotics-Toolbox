close all
clear

import mpr.*
ed = [3,2,1]/4;
m = 2; l = ones(m,1);
Qlim = [zeros(m,1),ones(m,1)*pi/2];
xi = twist(l);
Hb = transl([0.1,0.1,ed(3)/2]);
rob = MobileRobot(ed,xi,Hb,'name','rob1','qlim', Qlim);
rob.plot([0,0,ed(3)/2,0,0,0,0,0],'frame','workspace',[-1,3,-1,3,0,4]);
q0 = [0,0,ed(3)/2,0,0,0,0,0];
q1 = [1,0,ed(3)/2,0,0,-1,1,2];
q2 = [1,1,ed(3)/2,0,0,1,2,1];
[qq,~,~,tq] = calctraj([q0;q1;q2],0.1*ones(size(q0)),0.1,1);
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