function dx = xdot(t,x,siz,robot,lidar,tqr,qqr,dqqr,t0,Q0,dt,D,T_min,pfd,Qmax,val_max,var_max)

% x split to q
qfull = reshape(x,siz);
qrh = qfull(:,1:3);
qf = qfull(:,4:6);
lambda = qfull(:,7:9);
qrob = qfull(:,10:end);
% choose via point in time 't'
qr = interp1(tqr,qqr,t);
dqr = interp1(tqr,dqqr,t);
% robot num
robot_num = length(robot);
% robot controller - estimation
gamma = 1;
dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
% robot controller - tracking
kappa = 2;
dqf = dqrh-gamma*(qf-qrh)-kappa*D*(D'*qf);
% robot controller - constraint set and detector
if t-t0>=dt
    qf_next = qf + dqf*dt;
    Qnow = Q_updater(robot,lidar,qrob,qf,qf_next,pfd,Qmax,...
        val_max,var_max);
    t0 = t;
    Q0 = Qnow;
else
    Qnow = Q0;
end
% robot controller - optimization
[dqrob,dlambda] = robot_optimizer(robot,Qnow,D,T_min,pfd,qrob,lambda);
% q to x
dqfull = [dqrh,dqf,dlambda,dqrob];
dx = dqfull(:);
t
end