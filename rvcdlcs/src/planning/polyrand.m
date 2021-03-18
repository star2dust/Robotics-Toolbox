function [nodez,nodepoly,nodezinpoly] = polyrand(nodez,nodepoly,nodezinpoly,randnum,config,envir,robotnum,dispbaseref,dispendref)
% Generate random nodes and add it into the node set of a polygraph

% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% add nan between obstacles
obstpoly = envir.obstacle{1}';
for i=2:length(envir.obstacle)
    obstpoly = [obstpoly,NaN(2,1),envir.obstacle{i}'];
end
% generate nodes outside obstacles
ctr = 1; % counter
while (ctr <= randnum)
    % generate random two number in range of map's border
    randlocation = [rand* (envir.ub(1)-envir.lb(1)) + envir.lb(1);
        rand* (envir.ub(2)-envir.lb(2)) + envir.lb(2)]';
    % if this node is not inside any obstacle
    if ~inpolygon(randlocation(1),randlocation(2),obstpoly(1,:),obstpoly(2,:))
        % chech if inside a existing polytope
        if ~innodepoly(randlocation,nodepoly)
            % generate ploytope
            randpoly = polyinflate(envir.obstacle,randlocation,envir);
            % check whether formation exists in a polytope
            [randz,exit] = poly2nodez(randpoly,randlocation,config,envir,robotnum,dispbaseref,dispendref);
            if exit==1
                % add this location to nodelocation list
                nodez = [nodez;randz];
                nodepoly = [nodepoly,randpoly];
                nodezinpoly{polynum+ctr} = znum+ctr;
                %%%%%
%                 hold on
%                 plot(randlocation(1),randlocation(2),'go');
%                 plot(randz(1),randz(2),'r*');
%                 hold off
                %%%%%
                ctr=ctr+1;
            end
        end
    end
end
end

function flag = innodepoly(randlocation,nodepoly)
polynum = length(nodepoly);
count = 0;
for i=1:polynum
    if nodepoly(i).A*randlocation'-nodepoly(i).b<=0
        count = count+1;
    end
end
if count==0
    flag = false;
else
    flag = true;
end
end