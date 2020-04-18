function fig = robot_animator(fig,robot,lidar,qa,qb,qc,qrh,qf,Qnow,val_max,var_max,val_min,var_min)

% formation
pfd = fig.pfd; thfd = cart2pol(pfd(:,1),pfd(:,2));
thf = qf(:,3);
thd = thf+thfd;

for i=1:length(robot)
    robot(i).animate(qa(i,:),qb(i,:),fig.hrob(i).group);
    lidar(i).animate([qb(i,1:2),thd(i)],fig.hlid(i).group);
end

% ground pose 1x3 (qf: formation, qr: refernce, qrh: estimation of reference)
set(fig.hqf,'xdata',qf(:,1),'ydata',qf(:,2));
if ~isempty(fig.hqrh)
    set(fig.hqrh,'xdata',qrh(:,1),'ydata',qrh(:,2));
end

% air pose 1x6 (qc: object centroid)
if ~isempty(fig.hpc)
    set(fig.hpc,'xdata',qc(:,1),'ydata',qc(:,2),'zdata',qc(:,3));
end

% pb convhull
% iqb = convhull_(qb(:,1:2));
% set(fig.hb,'xdata',qb(iqb,1),'ydata',qb(iqb,2));

% sector space
s_max = Qnow.s_lim(2);
vsdal_min = (SO2(thf)*val_min')'/s_max;
vsdar_min = (SO2(thf)*var_min')'/s_max;
vsdal_max = (SO2(thf)*val_max')'/s_max;
vsdar_max = (SO2(thf)*var_max')'/s_max;
s_now = cell2mat_(Qnow.qlim,2,1);
psfd = s_now.*pfd;
psd = (SE2(qf)*psfd')';
Vda_max = [];
for i=1:length(robot)
    Vda = [qf(i,1:2);psd(i,:);psd(i,:)+s_now(i,:)*vsdal_max(i,:);
        psd(i,:)+s_now(i,:)*vsdar_max(i,:)];
    ida = convhull_(Vda);
    Vda_max = [Vda_max;Vda(ida,:)];
end
set(fig.hval_min,'XData',psd(:,1),'YData',psd(:,2),'UData',s_now.*vsdal_min(:,1),...
    'VData',s_now.*vsdal_min(:,2));
set(fig.hvar_min,'XData',psd(:,1),'YData',psd(:,2),'UData',s_now.*vsdar_min(:,1),...
    'VData',s_now.*vsdar_min(:,2));
% set(fig.hVda_max,'XData',Vda_max(:,1),'YData',Vda_max(:,2));

% psd convhull
isd = convhull_(psd);
set(fig.hpsd,'XData',psd(isd,1),'YData',psd(isd,2));
end