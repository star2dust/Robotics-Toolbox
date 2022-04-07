function [noderoute,polyroute] = polyastar(nodez,nodezinpoly,nodegrid,nodestart,nodegoal)
% Search a shortest path by A* in a polytope graph

% noderoute contains the start, goal and all via nodes
% calculated by A* algorithm
noderoute = astar(nodegrid,nodez,nodestart,nodegoal);
polynum = length(nodezinpoly);
ctr = 0; polyroute = [];
% polyroute consider each polytope as an edge between two adjacent nodes
% therefore length(polyroute) = length(noderoute)-1
for i=1:length(noderoute)-1
    for j=1:polynum
        % the bridge bewteen two adjacent nodes
        if find(nodezinpoly{j}==noderoute(i), 1).*find(nodezinpoly{j}==noderoute(i+1), 1)
            polyroute = [polyroute,j];
            ctr=ctr+1;
        end
    end
end