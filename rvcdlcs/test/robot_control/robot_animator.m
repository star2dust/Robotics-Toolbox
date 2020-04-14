function fig = robot_animator(fig,robot,lidar,qa,qb,qc,qrh,qf)

thfd = fig.qfd(:,3); pfd = fig.qfd(:,1:2);
s_min = fig.slim(1); s_max = fig.slim(2);
qfd_max = [s_max*pfd,thfd]; qfd_min = [s_min*pfd,thfd];
qd_max = q(SE2(qf)*SE2(qfd_max)); qd_min = q(SE2(qf)*SE2(qfd_min));

for i=1:length(robot)
    robot(i).animate(qa(i,:),qb(i,:),fig.hrob(i).group);
    lidar(i).animate([qb(i,1:2),thfd(i)],fig.hlid(i).group);
end

% ground pose 1x3 (qf: formation, qr: refernce, qrh: estimation of reference)
set(fig.hqf,'xdata',qf(:,1),'ydata',qf(:,2));
set(fig.hqrh,'xdata',qrh(:,1),'ydata',qrh(:,2));

% air pose 1x6 (qc: object centroid)
set(fig.hqc,'xdata',qc(:,1),'ydata',qc(:,2),'zdata',qc(:,3));

% iqb = convhull_(qb(:,1:2));
% set(fig.hb,'xdata',qb(iqb,1),'ydata',qb(iqb,2));
% 
% iqd = convhull_(pfd);
% set(fig.hd_max,'xdata',qd_max(iqd,1),'ydata',qd_max(iqd,2));
% set(fig.hd_min,'xdata',qd_min(iqd,1),'ydata',qd_min(iqd,2));
end