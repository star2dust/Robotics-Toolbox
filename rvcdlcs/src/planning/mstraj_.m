function [qt,qdt,qddt,tt] = mstraj_(qvia,qdmax,dt,tacc)
% MSTRAJ_ Calculate trajectory (q,qd,qdd) by qvia and qdmax
qlen = size(qvia,2);
q0 = qvia(1,:);
qt = mstraj(qvia(2:end,:),qdmax,[],q0,dt,tacc); % row vector for each xr
qt = [q0;qt];
qdt = [diff(qt)./dt;zeros(1,qlen)];
qddt = [diff(qdt)./dt;zeros(1,qlen)];
tt = 0:dt:(size(qt,1)-1)*dt;
end