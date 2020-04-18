close all
clear

% f1=imread('01.png');
% bw1=imbinarize(f1);%使用默认值0.5
% map = ~bw1(:,:,1);

scale = 0.5; cub_edge = [1,1,1]*scale; tile_edge = cub_edge(1:2);
[map, tile_q, tile_ind] = obs_read('01.png',tile_edge);
for i=1:length(tile_ind)
    tile_obs(i) = Cuboid2(tile_edge,'name',['tile' num2str(i)]);
end
figure;
for i=1:length(tile_ind)
    tile_obs(i).plot(tile_q(i,:),'workspace', [0 tile_edge(1)*size(map,1) 0 tile_edge(2)*size(map,2)],'facecolor','k');
    hold on;
end
view(2)
rob_loc = [10.2,10.3]; rob_r = 5;
% rob_sub = loc2sub(rob_loc,tile_edge);
% rob_sub_min = rob_sub; rob_sub_max = rob_sub; 
% rob_sub_min(1:2) = max(floor(rob_sub_min(1:2)-rob_r/scale),0*size(map));
% rob_sub_max(1:2) = min(ceil(rob_sub_max(1:2)+rob_r/scale),size(map));
% rob_ind_min = sub2ind(size_map,rob_sub_min(1),rob_sub_min(2));
% rob_ind_max = sub2ind(size_map,rob_sub_max(1),rob_sub_max(2));
% rob_tile_ind = []; rob_tile_q = [];
% for i=1:length(tile_ind)
%     if tile_ind(i)<=rob_ind_max&&tile_ind(i)>=rob_ind_min
%         if norm(tile_q(i,1:2)-rob_loc)<=5
%             rob_tile_ind = [rob_tile_ind,tile_ind(i)];
%             rob_tile_q = [rob_tile_q;tile_q(i,:)];
%         end
%     end
% end
% tile = Cuboid2(tile_edge); rob_obs_l = []; rob_obs_r = [];
d = [9,10];
% for i=1:length(rob_tile_ind)
%     if wedge2(d)*tile.vert(rob_tile_q(i,:))'>=0
%         rob_obs_l = [rob_obs_l;tile.vert(rob_tile_q(i,:))];
%     else
%         rob_obs_r = [rob_obs_r;tile.vert(rob_tile_q(i,:))];
%     end
% end
% kl = convhull(rob_obs_l(:,1),rob_obs_l(:,2));
% kr = convhull(rob_obs_r(:,1),rob_obs_r(:,2));
[obs_q, obs_ind] = obs_detect(tile_q,tile_ind,rob_loc,rob_r,map,tile_edge);
[obsl, obsr] = obs_divide(obs_q, obs_ind, d, tile_edge);
figure;
patch('vertices',obsl,'faces',1:size(obsl,1),'facecolor','y');
hold on
patch('vertices',obsr,'faces',1:size(obsr,1),'facecolor','g');
% for i=1:length(rob_tile_ind)
%     tile_obs(i) = Cuboid2(tile_edge,'name',['tile' num2str(i)]);
% end
% for i=1:length(rob_tile_ind)
%     tile_obs(i).plot(rob_tile_q(i,:),'workspace', [0 tile_edge(1)*size_map(1) 0 tile_edge(2)*size_map(2)],'facecolor','k');
%     hold on
% end
rob_circ = circle(rob_loc,rob_r);
plot(rob_circ(1,:),rob_circ(2,:),'r');
plot([0 d(1)], [0 d(2)], 'r');

var = [1,-3];
val = [-3,1];

[Ar,br] = ploycons(obsr);
[Al,bl] = ploycons(obsl);

slb = 0.9;
sub = 1.5;

% br = [br;sub;-slb];
% bl = [bl;sub;-slb];
% 
% Ar = [Ar;wedge2(var)./(wedge2(var)*d(:));-wedge2(var)./(wedge2(var)*d(:))];
% Al = [Al;wedge2(val)./(wedge2(val)*d(:));-wedge2(val)./(wedge2(val)*d(:))];


xl = linprog(wedge2(val)'./(wedge2(val)*d(:)),Al,bl);
xr = linprog(wedge2(var)'./(wedge2(var)*d(:)),Ar,br);
%lincon_plot(Ar,br,5,15)

plot(xl(1),xl(2),'yo');
plot(xr(1),xr(2),'go');
sal = wedge2(val)*xl./(wedge2(val)*d(:));
sar = wedge2(var)*xr./(wedge2(var)*d(:));
s = min(sal,sar);
plot([xr(1)-(sar-s)*d(1) s*d(1) xl(1)-(sal-s)*d(1)], [xr(2)-(sar-s)*d(2) s*d(2) xl(2)-(sal-s)*d(1)], 'r-.')
% plot([sal*d(1) xl(1)], [sal*d(2) xl(2)], 'y');
% plot([sar*d(1) xr(1)], [sar*d(2) xr(2)], 'g');



