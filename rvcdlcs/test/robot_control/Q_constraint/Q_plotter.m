function fig = Q_plotter(robot,lidar,qf,pfd,Qnow,val_max,var_max,val_min,var_min)

% load map file (unit: 1x1 formation = 4x4 robot = 8x8 tile)
load(['map' num2str(lidar(1).mapind) '.mat'],'map1','map2','map3','ws3d');

% path
goal_ind = length(map1.matrix(:));
pr_via = Map.ind2loc(map1.siz, map1.tile, map1.route{goal_ind});
if lidar(1).mapind==3
    pr_via = [pr_via(1,:);15,4;pr_via(2:end,:)];
    thr_via = [0;0;pi/6;pi/6;0];
    qr_via = [pr_via,thr_via];
else
    thr_via = zeros(size(pr_via,1),1);
    qr_via = [pr_via,thr_via];
end
[qqr,dqr,ddqr,tqr] = calctraj(qr_via,1*ones(1,size(qr_via,2)),0.1,2);

% formation
% thfd = cart2pol(pfd(:,1),pfd(:,2));
thf = qf(:,3);
% thd = thf+thfd;

% map plot
obs_height = robot(1).altitude+robot(1).height;
obs_thick = 1;
ws = ws3d(1:end-2);
dim = length(ws)/2;
map2.plot('workspace',ws,'dim',dim,'obheight',obs_height,...
    'obthick',obs_thick); hold on;
for i=1:length(map3.Voc)
    plot(map3.Voc{i}(:,1),map3.Voc{i}(:,2),'g');
end
% path plot
hqr = plot(qqr(:,1),qqr(:,2),'r');

% robot and lidar plot
% robot_lkthick = 2;
% robot_hgsize = 2;
% lidar_licolor = 'g';
% lidar_lithick = 0.4;
% for i=1:length(robot)
%     hrob(i) = robot(i).plot(qa(i,:),qb(i,:),'workspace',ws,'dim',dim,...
%         'plat','hgsize', robot_hgsize, 'lkthick', robot_lkthick);
%     hlid(i) = lidar(i).plot([qb(i,1:2),thd(i)],'workspace',ws,'dim',dim,...
%         'licolor',lidar_licolor,'lithick',lidar_lithick,'detect','nolidar');
% end

% ground pose 1x3 (qf: formation, qr: refernce, qrh: estimation of reference)
hqf = plot(qf(:,1),qf(:,2),'m*');
% if ~isempty(qrh)
%     hqrh = plot(qrh(:,1),qrh(:,2),'mo');
% else
%     hqrh = [];
% end

% air pose 1x6 (qc: object centroid)
% if ~isempty(qc)
%     hpc = plot3(qc(:,1),qc(:,2),qc(:,3),'rd');
% else
%     hpc = [];
% end

% pb convhull
% iqb = convhull_(qb(:,1:2));
% hqb = plot(qb(iqb,1),qb(iqb,2),'b');



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
hval_min = quiver(psd(:,1),psd(:,2),s_now.*vsdal_min(:,1),...
    s_now.*vsdal_min(:,2),'color','g','ShowArrowHead','off');
hvar_min = quiver(psd(:,1),psd(:,2),s_now.*vsdar_min(:,1),...
    s_now.*vsdar_min(:,2),'color','g','ShowArrowHead','off');
% hVda_max = plot(Vda_max(:,1),Vda_max(:,2),'b-');

% psd convhull
isd = convhull_(psd);
hpsd = plot(psd(isd,1),psd(isd,2),'b');

% auxiliary lines
% s_now = s_now*0.9;
% psfd = s_now.*pfd;
% psd = (SE2(qf)*psfd')';
% plot(psd(:,1),psd(:,2),'bo');
% psal = psd+s_now.*vsdal_max;
% plot(psal(:,1),psal(:,2),'mo');
% save data
hold off
% main handles
% fig.hrob = hrob;
% fig.hlid = hlid;
fig.hqr = hqr;
fig.hqf = hqf;
% auxiliary handles
% fig.hqrh = hqrh;
% fig.hpc = hpc;
% fig.hqb = hqb;
fig.hval_min = hval_min;
fig.hvar_min = hvar_min;
% fig.hVda_max = hVda_max;
fig.hpsd = hpsd;
% data 
fig.pfd = pfd;
fig.Qnow = Qnow;
fig.qqr = qqr;
fig.dqr = dqr;
fig.ddqr = ddqr;
fig.tqr = tqr;
end