close all
clear
import Map.*
tile = 1; siz = [10, 10, 10]/2; pct = 0.2; n = siz(1)*siz(2)*siz(3);
[obs, qobs, map] = environment(siz,tile,pct);
bigraph = map2gphA(map,0);
bisub = ind2sub_(size(map),1:n);
%% test astar
route1 = astar(bigraph,bisub,1,n);
for i=1:length(route1)
    rloc1(i,:) = ind2loc(siz,tile,route1(i));
end
%% test dijkstra
route2 = dijkstra(bigraph,bisub,1);
for i=1:length(route2{n})
    rloc2(i,:) = ind2loc(siz,tile,route2{n}(i));
end
%% save path
if ~exist('path_3D_example.mat', 'file')
    save('path_3D_example.mat','rloc1','rloc2','obs','qobs');
end   
%% plot
figure;
for i=1:length(obs)
   obs(i).plot(qobs(i,:),'workspace', [0 tile*siz(1) 0 tile*siz(2) 0 tile*siz(3)],'facecolor','k','facealpha',0.5);
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