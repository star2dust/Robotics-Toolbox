function fig = robot_animator(fig,robot,lidar,qrob,qrh,qf,Qnow,val_min,var_min,val_max,var_max)

% qrob split
if ~(isempty(qrob)||isempty(robot))
    [s,qfe,qae,qe,qa,qb,qc] = qrob_split(robot,qrob,qf);
else
    s = cell2mat_(Qnow.qlim,2,1); qa = []; qb = []; qc = [];
end

% lidar orientation
pfd = fig.pfd; 
phifd = cart2pol(pfd(:,1),pfd(:,2));
phif = qf(:,3);
phid = phif+phifd;

% robot and lidar plot
for i=1:fig.nrob
    if ~(isempty(qa)||isempty(robot))
        robot(i).animate(qa(i,:),qb(i,:),fig.hrob(i).group);
    end
    if ~isempty(qb)&&isa(lidar,'Lidar')
        lidar(i).animate([qb(i,1:2),phid(i)],fig.hlid(i).group);
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
% if ~isempty(qa)
%     for i=1:fig.nrob
%         Ve = (SE2(qf(i,:))*Qnow.Vfe{i}')';
%         set(fig.hQpe(i),'xdata',Ve(:,1),'ydata',Ve(:,2));
%     end
% end

% inner sector space
s_max = Qnow.s_lim(2);
vsdal_min = (SO2(phif)*val_min')'/s_max;
vsdar_min = (SO2(phif)*var_min')'/s_max;
vsdal_max = (SO2(phif)*val_max')'/s_max;
vsdar_max = (SO2(phif)*var_max')'/s_max;
psfd = s.*pfd;
psd = (SE2(qf)*psfd')';
set(fig.hval_min,'XData',psd(:,1),'YData',psd(:,2),'UData',s.*vsdal_min(:,1),...
    'VData',s.*vsdal_min(:,2));
set(fig.hvar_min,'XData',psd(:,1),'YData',psd(:,2),'UData',s.*vsdar_min(:,1),...
    'VData',s.*vsdar_min(:,2));


% diamond detection space for all robots
s_max_now = cell2mat_(Qnow.qlim,2,1,1);
psfd_max = s_max_now.*pfd;
psd_max = (SE2(qf)*psfd_max')';
if nargin<13||~(isempty(val_max)||isempty(val_max))
    for i=1:fig.nrob
        Vda_max = [qf(i,1:2);psd_max(i,:);psd_max(i,:)+s_max_now(i,:)*vsdal_max(i,:);
            psd_max(i,:)+s_max_now(i,:)*vsdar_max(i,:)];
        ida_max = convhull_(Vda_max);
        set(fig.hVda_max(i),'XData',Vda_max(ida_max,1),'YData',Vda_max(ida_max,2));
    end
end

% psd_max convhull
% isd_max = convhull_(psd_max);
% set(fig.hpsd_max,'XData',psd_max(isd_max,1),'YData',psd_max(isd_max,2));

% psd convhull
isd = convhull_(psd);
set(fig.hpsd,'XData',psd(isd,1),'YData',psd(isd,2));

% object shape
ipe = convhull_(qe(:,1:2));
set(fig.hpe,'XData',qe(ipe,1),'YData',qe(ipe,2));

end