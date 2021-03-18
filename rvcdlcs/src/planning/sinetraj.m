function [q,qd,qdd,t] = sinetraj(q0,qf,tmax,ymax,dt)
% Generate sine trajectory
if nargin<5
    ymax = 3; dt = 0.01;
end
yvel = 2*pi/tmax;
xvel = norm(q0-qf)/tmax; 
xang = cart2pol(qf(1)-q0(1),qf(2)-q0(2));
t = 0:dt:tmax; 
q = q0+[xvel*t(:),ymax*sin(yvel*t(:))]*rot2(xang)';
qd = [xvel*ones(length(t(:)),1),ymax*yvel*cos(yvel*t(:))]*rot2(xang)';
qdd = [0*ones(length(t(:)),1),-ymax*yvel^2*sin(yvel*t(:))]*rot2(xang)';

end