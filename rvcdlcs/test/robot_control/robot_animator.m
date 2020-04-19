function fig = robot_animator(fig,robot,lidar,qa,qb,qc,qrh,qf,Qnow,val_min,var_min,val_max,var_max)

% formation
pfd = fig.pfd; thfd = cart2pol(pfd(:,1),pfd(:,2));
thf = qf(:,3);
thd = thf+thfd;

% robot and lidar plot
for i=1:fig.nrob
    if ~(isempty(qa)||isempty(robot))
        robot(i).animate(qa(i,:),qb(i,:),fig.hrob(i).group);
    end
    if ~isempty(qb)&&isa(lidar,'Lidar')
        lidar(i).animate([qb(i,1:2),thd(i)],fig.hlid(i).group);
    end
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

% pfe circle constraint
if ~isempty(qa)
    for i=1:fig.nrob
        Ve = (SE2(qf(i,:))*Qnow.Vfe{i}')';
        set(fig.hVfe(i),'xdata',Ve(:,1),'ydata',Ve(:,2));
    end
end

% sector space
s_max = Qnow.s_lim(2);
vsdal_min = (SO2(thf)*val_min')'/s_max;
vsdar_min = (SO2(thf)*var_min')'/s_max;
vsdal_max = (SO2(thf)*val_max')'/s_max;
vsdar_max = (SO2(thf)*var_max')'/s_max;
s_max_now = cell2mat_(Qnow.qlim,2,1);
psfd_max_now = s_max_now.*pfd;
psd_max_now = (SE2(qf)*psfd_max_now')';
set(fig.hval_min,'XData',psd_max_now(:,1),'YData',psd_max_now(:,2),'UData',s_max_now.*vsdal_min(:,1),...
    'VData',s_max_now.*vsdal_min(:,2));
set(fig.hvar_min,'XData',psd_max_now(:,1),'YData',psd_max_now(:,2),'UData',s_max_now.*vsdar_min(:,1),...
    'VData',s_max_now.*vsdar_min(:,2));


% Vda for all robots
if nargin<13||~(isempty(val_max)||isempty(val_max))
    Vda_max = [];
    for i=1:fig.nrob
        Vda = [qf(i,1:2);psd_max_now(i,:);psd_max_now(i,:)+s_max_now(i,:)*vsdal_max(i,:);
            psd_max_now(i,:)+s_max_now(i,:)*vsdar_max(i,:)];
        ida = convhull_(Vda);
        Vda_max = [Vda_max;Vda(ida,:)];
    end
    set(fig.hVda_max,'XData',Vda_max(:,1),'YData',Vda_max(:,2));
end

% psd convhull
isd = convhull_(psd_max_now);
set(fig.hpsd,'XData',psd_max_now(isd,1),'YData',psd_max_now(isd,2));
end