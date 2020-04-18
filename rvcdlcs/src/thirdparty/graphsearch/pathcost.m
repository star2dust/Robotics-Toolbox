function cost=pathcost(route)
% calculate route cost
cost=0;
for i=1:size(route,1)-1
    cost=cost+costcal(route(i,:),route(i+1,:));
end

