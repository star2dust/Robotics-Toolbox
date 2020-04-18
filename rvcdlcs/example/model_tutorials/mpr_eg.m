close all
clear

m = 3; link = ones(m,1)*0.8; edge = [1,.8,.5]; mh = [0.2,0.2,1];
r = MobilePlanarRevolute(edge,link,mh,'name','rob1','type','elbowup');
r.plot([0,0,0,.3,.5,.2],'workspace',[-1 3 -1 3 0 4],'frame');


q0 = [0,0,0,0,0,0];
q1 = [1,0,0,-1,1,2];
q2 = [1,1,0,1,2,1];
[qq,~,~,tq] = calctraj([q0;q1;q2],0.1*ones(size(q0)),0.1,1);
tic;
playspeed = 2;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    r.animate(q);
    drawnow
end
toc