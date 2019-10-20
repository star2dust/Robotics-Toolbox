function [qarray, qdarray, qddarray, tf] = calctraj(qvia,qdmax,dt,tacc)
% calculate trajectory (q,qd,qdd) through qvia
qlen = size(qvia,2);
q0 = qvia(1,:);
qarray = mstraj(qvia(2:end,:),qdmax,[],q0,dt,tacc); % row vector for each xr
qarray = [q0;qarray];
qdarray = [diff(qarray)./dt;zeros(1,qlen)];
qddarray = [diff(qdarray)./dt;zeros(1,qlen)];
% tarray = 0:dt:(size(qarray,1)-1)*dt;
tf = (size(qarray,1)-1)*dt;
end