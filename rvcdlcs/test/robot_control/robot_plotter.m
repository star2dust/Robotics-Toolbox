function fig = robot_plotter(robot,lidar,ind,qa,qb,qc,qrh,qf,qfd,slim,val_min,var_min)

% load map file (unit: 1x1 formation = 4x4 robot = 8x8 tile)
load(['map' num2str(ind) '.mat'],'map1','map2','map3','ws3d');

% path
prvia = Map.ind2loc(map1.siz, map1.tile, map1.route{length(map1.matrix(:))});
qrvia = [prvia,zeros(size(prvia(:,1)))];
[qqr,dqr,ddqr,tqr] = calctraj(qrvia,1*ones(1,size(qrvia,2)),0.1,2);

% formation
thfd = qfd(:,3); pfd = qfd(:,1:2);
s_min = slim(1); s_max = slim(2);
qfd_max = [s_max*pfd,thfd]; qfd_min = [s_min*pfd,thfd];
qd_max = q(SE2(qf)*SE2(qfd_max)); qd_min = q(SE2(qf)*SE2(qfd_min));

% map plot
obs_height = robot(1).altitude+robot(1).height;
obs_thick = 1;
ws = ws3d(1:end-2);
map2.plot('workspace',ws,'dim',length(ws)/2,'obheight',obs_height,'obthick',obs_thick); hold on
for i=1:length(map3.Voc)
    plot(map3.Voc{i}(:,1),map3.Voc{i}(:,2),'g');
end
hqr = plot(qqr(:,1),qqr(:,2),'r');

% robot and lidar plot
robot_lkthick = 2;
robot_hgsize = 2;
lidar_licolor = 'g';
lidar_lithick = 0.4;
for i=1:length(robot)
    hrob(i) = robot(i).plot(qa(i,:), qb(i,:), 'workspace', ws, 'dim', length(ws)/2, 'plat',...
        'hgsize', robot_hgsize, 'lkthick', robot_lkthick);
    hlid(i) = lidar(i).plot(qd_min(i,:), 'workspace', ws, 'dim', length(ws)/2,...
        'licolor', lidar_licolor, 'lithick', lidar_lithick);
    quiver(qb(i,1),qb(i,2),val_min(i,1),val_min(i,2),'color','g');
    quiver(qb(i,1),qb(i,2),var_min(i,1),var_min(i,2),'color','g');
%     quiver(qb(i,1),qb(i,2),val_max(i,1),val_max(i,2),'color','m');
%     quiver(qb(i,1),qb(i,2),var_max(i,1),var_max(i,2),'color','m');
end

% ground pose 1x3 (qf: formation, qr: refernce, qrh: estimation of reference)
hqf = plot(qf(:,1),qf(:,2),'m*');
hqrh = plot(qrh(:,1),qrh(:,2),'mo');

% air pose 1x6 (qc: object centroid)
hqc = plot(qc(:,1),qc(:,2),'rd');

% auxiliary lines
% iqb = convhull_(qb(:,1:2));
% hd_max = plot(qd_max(iqb,1),qd_max(iqb,2),'g');
% hd_min = plot(qd_min(iqb,1),qd_min(iqb,2),'g');
% hb = plot(qb(iqb,1),qb(iqb,2),'b');

% save handles
hold off
fig.hrob = hrob;
fig.hlid = hlid;
fig.hqr = hqr;
fig.hqf = hqf;
fig.hqrh = hqrh;
fig.hqc = hqc;
% fig.hd_max = hd_max;
% fig.hd_min = hd_min;
% fig.hb = hb;
fig.qfd = qfd;
fig.slim = slim;
fig.qqr = qqr;
fig.dqr = dqr;
fig.ddqr = ddqr;
fig.tqr = tqr;
end