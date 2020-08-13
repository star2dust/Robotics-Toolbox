close all
clear

rob = Platform([3,2,1]/3,'name', 'rob');
qpf = [0,3,0];
qwh = [1,2,3,4];
dqpf = zeros(size(qpf));
dqwh = zeros(size(qwh));

q0 = [0,3,0];
q1 = [2,2,1];
q2 = [3,0,0];

[qq,dqq,~,tq] = mstraj_([q0;q1;q2],0.1*ones(size(q0)),0.1,1);

figure; 
rob.plot([qpf,qwh],'frame','workspace', [-0.5 3.5 -0.5 3.5 0 4]); hold on
plot(qq(:,1),qq(:,2),'r-');
rotate3d on
hold off

tic;
playspeed = 1;
dt = 0.01;
for tnow=0:dt:tq(end)
    % choose via point in time 'tnow'
    dq = interp1(tq,dqq,tnow); 
    % controller
    dqwh = dq*rob.jacob';
    % update object
    qwh = qwh+dt*dqwh;
    qpf = qpf+dt*dq;
    rob.animate([qpf,qwh]);
    drawnow
end