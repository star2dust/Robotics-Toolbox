close all 
clear

i = 3;
load(['Copy_of_map' num2str(i) '.mat'],'map_min', 'Af_min', 'route_min', 'tile_max');
% map to indoc/Voc
map_siz = size(map_min);
indoc_min = Map.map2indoc(map_min);
Voc_min = Map.indoc2Voc(map_siz,tile_max,indoc_min);
route_loc = Map.ind2loc(map_siz,tile_max,route_min);
for i=1:length(Voc_min)
    Ioc = boundary_(Voc_min{i},1);
    plot(Voc_min{i}(Ioc,1),Voc_min{i}(Ioc,2),'k'); hold on
    axis([0 tile_max*map_siz(1) 0 tile_max*map_siz(2)])
end
plot(route_loc(:,1),route_loc(:,2),'r');