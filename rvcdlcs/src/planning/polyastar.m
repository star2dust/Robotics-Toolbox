function [noderoute,polyroute] = polyastar(nodez,nodezinpoly,nodegrid,nodestart,nodegoal)
% Search a shortest path by astar in a polygraph
noderoute = astar(nodegrid,nodez,nodestart,nodegoal);
polynum = length(nodezinpoly);
ctr = 0; polyroute = [];
for i=1:length(noderoute)
    for j=1:polynum
        if ~isempty(find(nodezinpoly{j}==noderoute(i), 1))
            polyroute = [polyroute,j];
            ctr=ctr+1;
        end
    end
end