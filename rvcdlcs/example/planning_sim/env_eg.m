close all
clear

tile = 2; edge = [ones(1,2)*tile,0.5]; smap = [ones(1,2)*10,1]; pct = 0.2;
[obs, qobs, map] = environment(edge,smap,pct);
[bigraph,biloc] = map2graph(map);
n = smap(1)*smap(2)*smap(3);
%% test astar
route1 = astar(bigraph,biloc,1,n);
for i=1:length(route1)
    rloc1(i,:) = ind2loc(smap,route1(i),edge);
end
%% test dijkstra
route2 = dijkstra(bigraph,biloc,1);
for i=1:length(route2{n})
    rloc2(i,:) = ind2loc(smap,route2{n}(i),edge);
end
%% save path
if ~exist('path_example.mat', 'file')
    save('path_example.mat','rloc1','rloc2','obs','qobs');
end   
%% plot
figure;
for i=1:length(obs)
   obs(i).plot(qobs(i,:),'workspace', [0 edge(1)*smap(1) 0 edge(2)*smap(2)],'facecolor','k','facealpha',0.5);
   hold on; 
end
if ~isempty(route1)
    plot(rloc1(:,1),rloc1(:,2),'ro-');
end
if ~isempty(route2{n})
    plot(rloc2(:,1),rloc2(:,2),'bo-');
end
view(2); xlabel('x');ylabel('y');
hold off