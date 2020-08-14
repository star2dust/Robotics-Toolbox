close all
clear

rob = Platform([0.6 0.4 0.1],'name', 'rob','real3d');
qpf = [0,3,0];
qwh = [1,2,3,4];
dqpf = zeros(size(qpf));
dqwh = zeros(size(qwh));

q0 = [0,3,0];
q1 = [1,3,1];
q2 = [1,2,0];

[qq,dqq,~,tq] = mstraj_([q0;q1;q2],0.1*ones(size(q0)),0.1,1);

figure; 
rob.plot([qpf,qwh],'frame','workspace', [-0.5 1.5 1.5 3.5 0 2]); hold on
plot(qq(:,1),qq(:,2),'r-');
rotate3d on
hold off

tic;
playspeed = 4;
dt = 0.01;
ctr = 0;
for tnow=0:dt:tq(end)
    ctr = ctr+1;
    % choose via point in time 'tnow'
    dq = interp1(tq,dqq,tnow); 
    % controller
    dqwh = dq*rob.jacob';
    % update object
    qwh = qwh+dt*dqwh;
    qpf = qpf+dt*dq;
    if mod(ctr,playspeed)==0
        rob.animate([qpf,qwh]);
    end
    drawnow
end