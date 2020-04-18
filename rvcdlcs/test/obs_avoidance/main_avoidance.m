close all
clear


% unit: 1x1 formation = 4x4 robot = 8x8 tile
i = 3;
load(['map' num2str(i) '.mat'],'map1','map2','map3','ws3d');
% map data
tile_w = 1;
obs_height = tile_w;
obs_thick = 0.1;
eps_safe = 0.1;
% robot data
lidar_radius = 4;
lidar_licolor = 'g';
lidar_lithick = 0.4;
lidar_hlim = [0,obs_height];
link_num = 3;
robot_radius = tile_w-eps_safe;
robot_altitude = obs_height;
robot_height = tile_w;
robot_link = ones(1,link_num)*robot_height*0.95;
robot_lkthick = 2;
robot_hgsize = 2;
robot_num = 4;
% initial centrid and formation
pc = [4,4]; pcd = [-1,-1;1,-1;1,1;-1,1]*3;
qcd = [pcd,cart2pol(pcd(:,1),pcd(:,2))];
qc = [4,4,0]; qd = q(SE2(qc)*SE2(qcd));
s_min = 0.8; s_max = 1;
for i=2:robot_num+1
    I = convhull_(pcd); I = [I(end-1);I]; j = i-1;
    % l:left, r:right
    val_min(j,:) = s_min*pcd(I(i-1),:)-s_max*pcd(I(i),:);
    var_min(j,:) = s_min*pcd(I(i+1),:)-s_max*pcd(I(i),:);
    val_max(j,:) = s_max*pcd(I(i-1),:)-s_min*pcd(I(i),:);
    var_max(j,:) = s_max*pcd(I(i+1),:)-s_min*pcd(I(i),:);
    T_min{j} = [skew_(val_min(j,:))/norm(skew_(val_min(j,:)));skew_(var_min(j,:))/norm(skew_(var_min(j,:)))];
    T_max{j} = [skew_(val_max(j,:))/norm(skew_(val_max(j,:)));skew_(var_max(j,:))/norm(skew_(var_max(j,:)))];
    th1_min(j) = atan2(var_min(j,2),var_min(j,1));
    th1_max(j) = atan2(val_min(j,2),val_min(j,1));
    qlim{j} = [th1_min(j),zeros(1,link_num-1);th1_max(j),ones(1,link_num-1)*pi/2];
end
% lidar and robot
for i=1:robot_num
    lidar_object(i) = Lidar(lidar_radius,'name',['lidar' num2str(i)],'hlim',lidar_hlim);
    robot_object(i) = PlanarRevolute(robot_link,'name',['rob' num2str(i)], 'height', robot_height,...
        'radius', robot_radius, 'altitude', robot_altitude, 'qlim', qlim{i});
end

% initial pose
pb = [1,1; 7,1; 7,7; 1,7];
pe = [3,3; 5,3; 5,5; 3,5];
qb = [pb,zeros(robot_num,1)];
qe = [pe,ones(robot_num,1)*(robot_altitude+robot_height),zeros(robot_num,3)];
qa = zeros(robot_num,link_num); 
qce = zeros(robot_num,6); qcb = qce;
for i=1:robot_num
    qa(i,:) = robot_object(i).ikine(qe(i,:),qb(i,:));
    qce(i,:) = toqrpy(SE3(qc)^-1*SE3.qrpy(qe(i,:)));
end

% path
pcvia = Map.ind2loc(map1.siz, map1.tile, map1.route{length(map1.matrix(:))});
qcvia = [pcvia,zeros(size(pcvia(:,1)))];
svia = rand(size(qcvia,1),1)*(s_max-s_min)+s_min;
qcsvia = [qcvia,svia];
[qqc,~,~,tqc] = calctraj(qcsvia,1*ones(1,size(qcsvia,2)),0.1,2);
% [vxqc, vyqc] = pol2cart(qqc(:,3),ones(size(qqc(:,3))));

% figure
ws = ws3d(1:end-2);
map2.plot('workspace',ws,'dim',length(ws)/2); hold on
map3.plot('workspace',ws,'dim',length(ws)/2,'obcolor','r');
plot(qqc(:,1),qqc(:,2),'r');
% quiver(qqc(:,1),qqc(:,2),vxqc,vyqc,'color','r');
for i=1:robot_num
    hr(i) = robot_object(i).plot(qa(i,:), qb(i,:), 'workspace', ws, 'dim', length(ws)/2, 'plat',...
        'hgsize', robot_hgsize, 'lkthick', robot_lkthick);
    hl(i) = lidar_object(i).plot(qd(i,:), 'workspace', ws, 'dim', length(ws)/2,...
        'licolor', lidar_licolor, 'lithick', lidar_lithick);
%     quiver(qb(i,1),qb(i,2),val_min(i,1),val_min(i,2),'color','g');
%     quiver(qb(i,1),qb(i,2),var_min(i,1),var_min(i,2),'color','g');
%     quiver(qb(i,1),qb(i,2),val_max(i,1),val_max(i,2),'color','m');
%     quiver(qb(i,1),qb(i,2),var_max(i,1),var_max(i,2),'color','m');
end

% write video
video_on = false;
if video_on
    respath = 'D:\Users\Woody\Documents\MATLAB\';
    videoname = [respath 'story_demo'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end

tic;
playspeed = 2;
while toc<tqc(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    qcs = interp1(tqc,qqc,tnow);
    qc = qcs(:,1:end-1); s = qcs(:,end);
    % update object
    for i=1:robot_num
        qb(i,:) = q(SE2(qc)*SE2([qcd(i,1:2)*s,qcd(i,3)])); qb(i,3) = 0;
        qe(i,:) = toqrpy(SE3(qc)*SE3.qrpy(qce(i,:)));
        qa(i,:) = robot_object(i).ikine(qe(i,:),qb(i,:));
        qd = q(SE2(qc)*SE2(qcd));
        robot_object(i).animate(qa(i,:),qb(i,:),hr(i).group);
        lidar_object(i).animate(qd(i,:),hl(i).group); 
    end 
    % video
    if video_on
        f = getframe(gcf);
        f.cdata = imresize(f.cdata,[480,640]);
        writeVideo(writerObj,f);
    end
    drawnow
end
toc
if video_on
    close(writerObj);
end