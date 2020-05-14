function fig = robot_plotter(robot,lidar,qrob,qrh,qf,pfd,Qnow,val_min,var_min,val_max,var_max)

% qrob split
if ~(isempty(qrob)||isempty(robot))
    [s,qfe,qae,qe,qa,qb,qc] = qrob_split(robot,qrob,qf);
else
    s = cell2mat_(Qnow.qlim,2,1); qa = []; qb = []; qc = [];
end

% map and lidar 
if isa(lidar,'Lidar')   
    map_ind = lidar(1).mapind;
else
    map_ind = 3;
end

% load map file (unit: 1x1 formation = 4x4 robot = 8x8 tile)
load(['map' num2str(map_ind) '.mat'],'map1','map2','map3','ws3d');

% map plot
obs_height = robot(1).altitude+robot(1).height;
obs_thick = 1;
ws = ws3d(1:end-2);
% ws = [0 8 0 8];
dim = length(ws)/2;
if isa(lidar,'Lidar')
    map2.plot('workspace',ws,'dim',dim,'obheight',obs_height,...
        'obthick',obs_thick); hold on
    for i=1:length(map3.Voc)
        plot(map3.Voc{i}(:,1),map3.Voc{i}(:,2),'g');
    end
end

% lidar orientation
phifd = cart2pol(pfd(:,1),pfd(:,2));
phif = qf(:,3);
phid = phif+phifd;

% robot and lidar plot
robot_lkthick = 2;
robot_hgsize = 2;
lidar_licolor = 'g';
lidar_lithick = 0.4;
nrob = length(robot);
for i=1:nrob
    if ~(isempty(qa)||isempty(robot))
        hrob(i) = robot(i).plot(qa(i,:),qb(i,:),'workspace',ws,'dim',dim,...
            'plat','hgsize', robot_hgsize, 'lkthick', robot_lkthick); hold on
    else
        hrob = [];
    end
    if ~isempty(qb)&&isa(lidar,'Lidar')
        hlid(i) = lidar(i).plot([qb(i,1:2),phid(i)],'workspace',ws,'dim',dim,...
            'licolor',lidar_licolor,'lithick',lidar_lithick,'detect','nolidar');
        hold on
    else
        hlid = [];
    end
end


% path
goal_ind = length(map1.matrix(:));
pr_via = Map.ind2loc(map1.siz, map1.tile, map1.route{goal_ind});
if map_ind==3
    pr_via = [pr_via(1,:);15,4;pr_via(2:end,:)];
    phir_via = [0;0;pi/6;pi/6;0];
    qr_via = [pr_via,phir_via];
else
    phir_via = zeros(size(pr_via,1),1);
    qr_via = [pr_via,phir_via];
end
[qqr,dqr,ddqr,tqr] = calctraj(qr_via,1*ones(1,size(qr_via,2)),0.2,2);

% path plot
hqr = plot(qqr(:,1),qqr(:,2),'r');

% ground pose 1x3 (qf: formation, qr: refernce, qrh: estimation of reference)
hqf = plot(qf(:,1),qf(:,2),'m*');
if ~isempty(qrh)
    hqrh = plot(qrh(:,1),qrh(:,2),'mo');
else
    hqrh = [];
end

% air pose 1x6 (qc: object centroid)
if ~isempty(qc)
    hpc = plot3(qc(:,1),qc(:,2),qc(:,3),'rd');
else
    hpc = [];
end

% pfe circle constraint
% if ~isempty(qa)
%     for i=1:nrob
%         Ve = (SE2(qf(i,:))*Qnow.Vfe{i}')';
%         hQpe(i) = plot(Ve(:,1),Ve(:,2),'m');
%     end
% else
%     hQpe = [];
% end

% inner sector space
s_max = Qnow.s_lim(2);
vsdal_min = (SO2(phif)*val_min')'/s_max;
vsdar_min = (SO2(phif)*var_min')'/s_max;
vsdal_max = (SO2(phif)*val_max')'/s_max;
vsdar_max = (SO2(phif)*var_max')'/s_max;
psfd = s.*pfd;
psd = (SE2(qf)*psfd')';
hval_min = quiver(psd(:,1),psd(:,2),s.*vsdal_min(:,1),...
    s.*vsdal_min(:,2),'color','g','ShowArrowHead','off');
hvar_min = quiver(psd(:,1),psd(:,2),s.*vsdar_min(:,1),...
    s.*vsdar_min(:,2),'color','g','ShowArrowHead','off');

% diamond detection space for all robots
s_max_now = cell2mat_(Qnow.qlim,2,1,1);
psfd_max = s_max_now.*pfd;
psd_max = (SE2(qf)*psfd_max')';
if ~(isempty(val_max)||isempty(var_max))
    for i=1:nrob
        Vda_max = [qf(i,1:2);psd_max(i,:);psd_max(i,:)+s_max_now(i,:)*vsdal_max(i,:);
            psd_max(i,:)+s_max_now(i,:)*vsdar_max(i,:)];
        ida_max = convhull_(Vda_max);
        hVda_max(i) = plot(Vda_max(ida_max,1),Vda_max(ida_max,2),'b:');
    end
else
    hVda_max = [];
end

% psd_max convhull
% isd_max = convhull_(psd_max);
% hpsd_max = plot(psd_max(isd_max,1),psd_max(isd_max,2),'b');

% psd convhull
isd = convhull_(psd);
hpsd = plot(psd(isd,1),psd(isd,2),'b');

% object shape
ipe = convhull_(qe(:,1:2));
hpe = plot(qe(ipe,1),qe(ipe,2),'c');


% save data
hold off
% main handles
fig.hrob = hrob;
fig.hlid = hlid;
fig.hqr = hqr;
fig.hqf = hqf;
% auxiliary handles
fig.hqrh = hqrh;
fig.hpc = hpc;
fig.hpe = hpe;
% fig.hQpe = hQpe;
fig.hval_min = hval_min;
fig.hvar_min = hvar_min;
fig.hVda_max = hVda_max;
% fig.hpsd_max = hpsd_max;
fig.hpsd = hpsd;
% data 
fig.nrob = nrob;
fig.pfd = pfd;
fig.Qnow = Qnow;
fig.qqr = qqr;
fig.dqr = dqr;
fig.ddqr = ddqr;
fig.tqr = tqr;
end