close all
clear

import Map.*

tile = 2; range = ones(1,2)*10; pct = 0.2;
map = randMap(range,pct);
n = length(map(:)); d = length(size(map)); s = size(map);
bigraph = map2gphA(map,0);
bisub = ind2sub_(size(map),1:n);

%% test astar
route1 = astar(bigraph,bisub,1,n);
for i=1:length(route1)
    rloc1(i,:) = ind2loc(s,tile,route1(i));
end
%% test dijkstra
route2 = dijkstra(bigraph,bisub,1);
for i=1:length(route2{n})
    rloc2(i,:) = ind2loc(s,tile,route2{n}(i));
end
%% save path
if ~exist('path_example.mat', 'file')
    save('path_example.mat','rloc1','rloc2','map','tile');
end   
%% plot
m = Map(map,'name','map1','tile',tile);
figure;
m.plot; hold on;
if ~isempty(route1)
    plot(rloc1(:,1),rloc1(:,2),'ro-');
end
if ~isempty(route2{n})
    plot(rloc2(:,1),rloc2(:,2),'bo-');
end
view(2); xlabel('x');ylabel('y');
hold off