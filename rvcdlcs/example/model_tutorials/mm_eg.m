close all
clear

% dh = [theta d a alpha]
edge = [0.8 0.6 0.4]*2; lx = 1; tool = SE3;
dh = [0,0,0,-pi/2,1;
	-pi/2,0,0,pi/2,1;
	0,edge(3)/2,0,0,0;
    0,lx,lx,0,0;
    0,0,lx*2,0,0;
    0,0,lx,0,0];
rob = MobileManipulator(edge,dh,tool,'name','rob1');
rob.plot([0,0,0,0,0,0],'frame','workspace',[-1,3,-1,3,0,4]*2);
q0 = [0,0,0,0,0,0];
q1 = [1,0,-1,1,2,1];
q2 = [1,1,1,2,1,0];
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