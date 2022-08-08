close all
clear

rob = Platform([0.114 0.114 0.06],'name','rob','r', 0.1754,'h',0.2193);
qpf = [0,3,0]*0.5;
qwh = [1,2,3,4];
dqpf = zeros(size(qpf));
dqwh = zeros(size(qwh));

q0 = [0,3,0]*0.5;
q1 = [1,3,1]*0.5;
q2 = [1,2,0]*0.5;

[qq,dqq,~,tq] = mstraj_([q0;q1;q2],0.1*ones(size(q0)),0.1,1);

figure; 
rob.plot([qpf,qwh],'frame','workspace', [-0.5 1.5 1.5 3.5 0 2]*0.5); hold on
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
    %dq = interp1(tq,dqq,tnow); 
    dq = [0,0,1];
    % controller
    dqwh = dq*rob.bwdkine';
    % update object
    qwh = qwh+dt*dqwh;
    qpf = qpf+dt*dq;
    if mod(ctr,playspeed)==0
        rob.animate([qpf,qwh]);
    end
    drawnow
end