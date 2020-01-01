close all
clear

ed = [3,2,1]/4;
m = 2; l = ones(m,1);
xi = twist(l);
rob = MobileRobot(ed,xi,'name','rob1');
rob.plot([0,0,0,0,0,0,0,0],'frame','workspace',[-1,3,-1,3,-1,1]);
q0 = [0,0,0,0,0,0,0,0];
q1 = [1,1,0,0,0,-1,1,2];
q2 = [1,1,0,0,0,1,2,1];
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