function [map1, map2, map3, ws3d] = map_reader(i)
% load(['map' num2str(i) '.mat'],'map_min','tile_max');
% Map object
% map = Map(map_min,'name',['map' num2str(i)],'tile',tile_max);
load(['map' num2str(i) '.mat'],'map1','map2','map3','ws3d');
figure
map1.plot('workspace', ws3d, 'dim', length(ws3d)/2, 'route');
figure
map2.plot('workspace', ws3d, 'dim', length(ws3d)/2);
figure
map3.plot('workspace', ws3d, 'dim', length(ws3d)/2);
% map to indoc/Voc
% map_siz = size(map_min);
% indoc_min = Map.map2indoc(map_min);
% Voc_min = Map.indoc2Voc(map_siz,tile_max,indoc_min);
% route_loc = Map.ind2loc(map_siz,tile_max,route_min);
% for i=1:length(map.Voc)
%     plot(map.Voc{i}(:,1),map.Voc{i}(:,2),'k'); hold on
%     axis([0 map.tile*map.siz(1) 0 map.tile*map.siz(2)])
% end
% n = length(map.matrix(:));
% locru = Map.ind2loc(map.siz,map.tile,map.route{n});
% plot(locru(:,1),locru(:,2),'r');
% hold off
% map.size = map_siz;
% map.tile = tile_max;
% map.matrix = map_min;
% map.Af = Af_min;
% map.locru = route_loc;
% map.indoc = indoc_min;
% map.Voc = Voc_min;
end