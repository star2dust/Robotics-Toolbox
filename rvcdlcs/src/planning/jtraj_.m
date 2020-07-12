function [qt,qdt,qddt,tt] = jtraj_(q0,q1,dt,qd0,qd1)
% JTRAJ_ Calculate trajectory (q,qd,qdd) by q0 and q1
if nargin == 3
    qd0 = zeros(size(q0));
    qd1 = qd0;
elseif nargin == 5
    qd0 = qd0(:);
    qd1 = qd1(:);
else
    error('incorrect number of arguments')
end
[qt,qdt,qddt] = jtraj(q0, q1, dt, qd0, qd1);
tt = 0:dt:(size(qt,1)-1)*dt;
end