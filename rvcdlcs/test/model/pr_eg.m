close all
clear

m = 3; link = ones(m,1)*0.8; edge = [1,.8,.5]; mh = [0.2,0.2,1];
r = PlanarRevolute(link,'name','rob1','height',1);

% r.plot([.3,.5,.2],-edge/2,'workspace',[-1 3 -1 3 0 4],'frame','hingestyle','o');
qb = [edge,0,0,0.5]; qa1 = [.3,.5,.2];
figure
r.plot(qa1,qb,'workspace',[-1 3 -1 3 0 4],'frame','platform','platformview',2,'hingestyle','o');
fk = r.fkine(qa1,qb);
qa2 = r.ikine(fk(end),qb);
figure
r.plot(qa2,qb,'workspace',[-1 3 -1 3 0 4],'frame','platform','platformview',2,'hingestyle','o');

r.getMu(r.link,qa1)
r.getMu(r.link,qa2)


q0 = [0,0,0,0,0,1/2,0,0,0];
q1 = [-1,1,2,1,0,1/2,0,0,1];
q2 = [1,2,1,1,1,1/2,0,0,0];
[qq,~,~,tq] = calctraj([q0;q1;q2],0.1*ones(size(q0)),0.1,1);

figure
r.plot(q0(1:3),q0(4:end),'workspace',[-1 3 -1 3 0 4],'frame','platform','platformview',3,'hingestyle','o');
tic;
playspeed = 2;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    r.animate(q(1:3),q(4:end));
    drawnow
end
toc