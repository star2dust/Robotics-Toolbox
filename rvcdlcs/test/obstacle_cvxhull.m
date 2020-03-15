close all
clear

f1=imread('01.png');
bw1=imbinarize(f1);%使用默认值0.5
map = ~bw1(:,:,1);

scale = 0.5; cub_edge = [1,1,1]*scale; tile_edge = cub_edge(1:2);
size_map = size(map);
[tile_q, tile_ind] = map2cub(map,tile_edge);
% for i=1:length(tile_ind)
%     tile_obs(i) = Cuboid2(tile_edge,'name',['tile' num2str(i)]);
% end
% figure;
% for i=1:length(tile_ind)
%     tile_obs(i).plot(tile_q(i,:),'workspace', [0 tile_edge(1)*size_map(1) 0 tile_edge(2)*size_map(2)],'facecolor','k');
%     hold on;
% end
% view(2)
rob_loc = [10.2,10.3]; rob_r = 5;
rob_sub = loc2sub(rob_loc,tile_edge);
rob_sub_min = rob_sub; rob_sub_max = rob_sub; 
rob_sub_min(1:2) = max(floor(rob_sub_min(1:2)-rob_r/scale),0*size(map));
rob_sub_max(1:2) = min(ceil(rob_sub_max(1:2)+rob_r/scale),size(map));
rob_ind_min = sub2ind(size_map,rob_sub_min(1),rob_sub_min(2));
rob_ind_max = sub2ind(size_map,rob_sub_max(1),rob_sub_max(2));
rob_tile_ind = []; rob_tile_q = [];
for i=1:length(tile_ind)
    if tile_ind(i)<=rob_ind_max&&tile_ind(i)>=rob_ind_min
        if norm(tile_q(i,1:2)-rob_loc)<=5
            rob_tile_ind = [rob_tile_ind,tile_ind(i)];
            rob_tile_q = [rob_tile_q;tile_q(i,:)];
        end
    end
end
figure;
for i=1:length(rob_tile_ind)
    tile_obs(i) = Cuboid2(tile_edge,'name',['tile' num2str(i)]);
end
for i=1:length(rob_tile_ind)
    tile_obs(i).plot(rob_tile_q(i,:),'workspace', [0 tile_edge(1)*size_map(1) 0 tile_edge(2)*size_map(2)],'facecolor','k');
    hold on
end
rob_circ = circle(rob_loc,rob_r);
plot(rob_circ(1,:),rob_circ(2,:),'r');

