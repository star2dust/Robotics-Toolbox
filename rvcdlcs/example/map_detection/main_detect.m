close all
clear

import Map.*
% map data
tile_w = 1; % cover one platform
room_w = 20;
obs_height = tile_w;
obs_thick = 0.1;
dilate_rad = 1;
map_siz = ceil([room_w/tile_w,room_w/tile_w]);
ws_dim = 3;
ws_range = [0 tile_w*map_siz(1) 0 tile_w*map_siz(2) 0 tile_w*sum(map_siz)/2];
eps_safe = 0.1;
% data
lidar_radius = 4;
lidar_licolor = 'g';
lidar_lithick = 0.4;
lidar_hlim = [0,obs_height];
link_num = 3;
robot_radius = tile_w-eps_safe;
robot_altitude = obs_height;
robot_height = tile_w;
robot_link = ones(1,link_num)*robot_height;
robot_lkthick = 2;
robot_hgsize = 2;
% read map
if ~exist('map.mat', 'file')
map_matrix = readMap('map_input.png');
map_dilated = dilateMap(map_matrix,'sphere',dilate_rad);
tic
map2 = Map(map_matrix,'name','map0','tile',tile_w);
map3 = Map(map_dilated,'name','map1','tile',tile_w);
toc
    save('map.mat','map2','map3');
end   
load('map.mat','map2','map3');
% lidar and robot
lidar_object = Lidar(lidar_radius,'name','lidar1','hlim',lidar_hlim);
robot_object = PlanarRevolute(robot_link,'name','rob1', 'height', robot_height,...
    'radius', robot_radius, 'altitude', robot_altitude);

% astar
siz = size(map3.matrix);
n = length(map3.matrix(:));
bigraph = map3.Af;
bisub = ind2sub_(siz,1:n);
route = astar(bigraph,bisub,1,n);
rloc = ind2loc(siz,tile_w,route);

qbvia = [rloc,rand(size(rloc(:,1)))];
qavia = rand(size(rloc,1),link_num); 

qvia = [qavia,qbvia];
[qq,~,~,tq] = mstraj_(qvia,1*ones(1,size(qvia,2)),0.1,2);

lazel = size(qq,1);
cazel = [-40 30;0 90;-40 30];
qazel = [(cazel(1,1):(cazel(2,1)-cazel(1,1))/160:cazel(2,1))',(cazel(1,2):(cazel(2,2)-cazel(1,2))/160:cazel(2,2))';
    kron(ones(50,1),cazel(2,:));
    (cazel(2,1):(cazel(3,1)-cazel(2,1))/(lazel-212):cazel(3,1))',(cazel(2,2):(cazel(3,2)-cazel(2,2))/(lazel-212):cazel(3,2))'];


figure
map2.plot('workspace', ws_range, 'dim', ws_dim, 'obheight', obs_height); hold on
map3.plot('workspace', ws_range, 'dim', ws_dim, 'obheight', obs_height, 'obthick', obs_thick); 
hr = robot_object.plot(qavia(1,:), qbvia(1,:), 'plat', 'dim', ws_dim,...
    'hgsize', robot_hgsize, 'lkthick', robot_lkthick);
hl = lidar_object.plot(qbvia(1,:), 'dim', ws_dim, 'detect',...
    'licolor', lidar_licolor, 'lithick', lidar_lithick);
plot(qq(:,link_num+1),qq(:,link_num+2),'m-','LineWidth',2);

% write video
video_on = false;
if video_on
    respath = 'D:\Users\Woody\Documents\MATLAB\';
    videoname = [respath 'story_demo'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end

tic;
playspeed = 0.5;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    qa = q(1:3); qb = q(4:end);
    robot_object.animate(qa,qb,hr.group);
    lidar_object.animate(qb,hl.group);
    azel = interp1(tq,qazel,tnow); 
    view(azel(1),azel(2));
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