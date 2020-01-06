close all
clear

tile = 1; edge = ones(1,3)*tile; gap = 2; smap = [10, 10, 10]/2; pct = 0.2;
[obs, qobs, map, mapg] = environment_gap(edge,smap,pct,gap);
[bigraph,biloc] = map2graph(map);
smapg = size(mapg);
n = smap(1)*smap(2)*smap(3);
%% test astar
route1 = astar(bigraph,biloc,1,n);
for i=1:length(route1)
    rloc1(i,:) = ind2loc(smap,route1(i),edge*gap);
end
%% test dijkstra
route2 = dijkstra(bigraph,biloc,1);
for i=1:length(route2{n})
    rloc2(i,:) = ind2loc(smap,route2{n}(i),edge*gap);
end
%% save path
if ~exist('path_gap_3D_example.mat', 'file')
    save('path_gap_3D_example.mat','rloc1','rloc2','obs','qobs');
end   
%% plot
figure;
for i=1:length(obs)
   obs(i).plot(qobs(i,:),'workspace', [0 edge(1)*smapg(1) 0 edge(2)*smapg(2) 0 edge(3)*smapg(3)],'facecolor','k','facealpha',0.5);
   hold on; 
end
if ~isempty(route1)
    plot3(rloc1(:,1),rloc1(:,2),rloc1(:,3),'ro-');
end
if ~isempty(route2{n})
    plot3(rloc2(:,1),rloc2(:,2),rloc2(:,3),'bo-');
end
view(3); xlabel('x');ylabel('y');zlabel('z');
hold off