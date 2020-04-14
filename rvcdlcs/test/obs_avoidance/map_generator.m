close all 
clear

% mat_siz = [4 4]; tile_max = 8;
% % path planning (astar)
% route_min = [];
% while isempty(route_min) % ensure route exists
%     mat_min = Map.randMap(mat_siz,0.3);
%     n = length(mat_min(:));
%     Af_min = Map.map2gphA(mat_min,0);
%     sub = ind2sub_(mat_siz,1:n);
%     route_min = astar(Af_min,sub,1,n);
% end
i = 3;
load(['data/Copy_of_map' num2str(i) '.mat'],'map_min','tile_max');
mat_min = map_min;
% save map 
% for i=0:9
if ~exist(['map' num2str(i) '.mat'], 'file')
    map1 = Map(mat_min,'name',['map' num2str(i)],'tile',tile_max);
    ws3d = [0 map1.tile*map1.siz(1) 0 map1.tile*map1.siz(2) 0 map1.tile*sum(map1.siz)/2];
    tile2 = 1; k12 = map1.tile/tile2;
    mat2 = imresize(map1.matrix,k12,'nearest');
    map2 = Map(mat2,'name',['map' num2str(i) '-resize'],'tile',tile2,'noplanning');
    mat3 = Map.dilateMap(mat2,'sphere',1);
    map3 = Map(mat3,'name',['map' num2str(i) '-dilated'],'tile',tile2,'noplanning');
    save(['map' num2str(i) '.mat'],'map1', 'map2', 'map3', 'ws3d');
    j = i;
%     break;
end
% end
% read map
map_reader(j);