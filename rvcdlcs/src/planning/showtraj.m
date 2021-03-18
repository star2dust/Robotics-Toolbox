function fig = showtraj(q,qd,t,ws)
% Show 2D trajectory

figure
fig.traj = plot(q(:,1),q(:,2)); hold on
if nargin<4
    axis equal
else
    axis(ws)
end
p = q(1,:);
fig.point = plot(p(1),p(2),'o');
dt = diff(t);
for i=1:size(q,1)-1
    p = p + qd(i,:)*dt(i);
    set(fig.point,'xdata',p(1),'ydata',p(2));
    drawnow
end